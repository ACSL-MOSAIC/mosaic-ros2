//
// Created by yhkim on 1/19/26.
//

#include "mosaic-ros2-sensor/util/progressive_point_cloud2_sender.hpp"
#include "protobuf/pcc_chunk.pb.h"

#include "../util/clock_utils.hpp"
#include "../util/uuid_utils.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

#include <mosaic/handlers/data_channel/data_channel_handler.hpp>

using namespace mosaic::ros2::sensor_connector;

void ProgressivePointCloud2Sender::SetParams(const std::unordered_map<std::string, std::string> &params) {
    if (params.find("voxel_size") != params.end()) {
        voxel_size_ = std::stof(params.at("voxel_size"));
    }
}

void ProgressivePointCloud2Sender::ProcessAsync(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    std::thread([this, msg] {
        this->SendInternal(msg);
    }).detach();
}

void ProgressivePointCloud2Sender::SendInternal(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    {
        std::lock_guard lock(mutex_);
        if (try_sending == true) {
            MOSAIC_LOG_ERROR("Frame drop!");
            return;
        }
        try_sending = true;

        // Initialize the first frame
        if (!initialized_) {
            config_ = std::make_unique<PointCloudConfig>();
            Initialize(msg);
        }

        if (!IsAllChannelReady()) {
            MOSAIC_LOG_VERBOSE("Not all channel ready!");
            try_sending = false;
            return;
        }
    }

    const long created_timestamp = utils::GetNow();

    // Create and send metadata first
    const auto ppc_meta = ExtractMeta(msg);
    const std::string frame_id = ppc_meta->frame_id();
    SendData(ppc_meta->SerializeAsString());

    // Build octree
    const auto octree_root = BuildOctree(msg);

    // Collect leaf nodes
    std::vector<OctreeNode *> leaf_nodes;
    CollectLeafNodes(octree_root.get(), leaf_nodes);

    SendPoints(msg, leaf_nodes, frame_id, created_timestamp);

    {
        std::lock_guard lock(mutex_);
        try_sending = false;
    }
}

void ProgressivePointCloud2Sender::SendPoints(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                                              const std::vector<OctreeNode *> &leaf_nodes, const std::string &frame_id,
                                              const long timestamp) {
    const uint32_t point_step = msg->point_step;
    const uint8_t *data_ptr = msg->data.data();

    size_t chunk_idx = 0;
    size_t total_points_sent = 0;
    const size_t total_points = msg->width * msg->height;

    // Loop until all points are sent
    while (total_points_sent < total_points) {
        bool has_remaining_points = true;
        const std::vector<uint8_t> chunk_buffer = BuildChunkBuffer(leaf_nodes, data_ptr, &total_points_sent,
                                                                   &has_remaining_points, point_step);

        BuildChunkAndSend(chunk_buffer, frame_id, &chunk_idx, timestamp, point_step);

        if (!has_remaining_points) {
            break;
        }
    }
}

std::vector<uint8_t> ProgressivePointCloud2Sender::BuildChunkBuffer(const std::vector<OctreeNode *> &leaf_nodes,
                                                                    const uint8_t *data_ptr, size_t *total_points_sent,
                                                                    bool *has_remaining_points,
                                                                    const uint32_t point_step) const {
    std::vector<uint8_t> chunk_buffer;
    chunk_buffer.reserve(config_->max_points_per_chunk * point_step);
    size_t points_in_chunk = 0;

    // Collect points from leaf nodes until chunk is full
    while (points_in_chunk < config_->max_points_per_chunk && *has_remaining_points) {
        *has_remaining_points = false;

        // Iterate through each leaf node
        for (auto *leaf: leaf_nodes) {
            // If this leaf still has points
            if (!leaf->point_indices.empty()) {
                *has_remaining_points = true;

                // Extract one point from the back (pop_back is efficient)
                const size_t point_idx = leaf->point_indices.back();
                leaf->point_indices.pop_back();

                // Add point data to chunk buffer
                const uint8_t *point_ptr = data_ptr + point_idx * point_step;
                chunk_buffer.insert(chunk_buffer.end(), point_ptr, point_ptr + point_step);

                ++points_in_chunk;
                ++*total_points_sent;

                // Stop if chunk is full
                if (points_in_chunk >= config_->max_points_per_chunk) {
                    break;
                }
            }
        }
    }

    return chunk_buffer;
}

void ProgressivePointCloud2Sender::BuildChunkAndSend(std::vector<uint8_t> chunk_buffer, const std::string &frame_id,
                                                     size_t *chunk_idx, const long timestamp,
                                                     const uint32_t point_step) {
    // Send chunk if not empty
    if (!chunk_buffer.empty()) {
        const size_t chunk_point_count = chunk_buffer.size() / point_step;

        ProgressivePointCloud::Chunk chunk;
        chunk.set_timestamp(timestamp);
        chunk.set_frame_id(frame_id);
        chunk.set_chunk_index(*chunk_idx);
        chunk.set_point_size(chunk_point_count);
        chunk.set_data(chunk_buffer.data(), chunk_buffer.size());

        SendData(chunk.SerializeAsString());

        ++*chunk_idx;
    }
}

std::shared_ptr<ProgressivePointCloud::Meta> ProgressivePointCloud2Sender::ExtractMeta(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) const {
    auto meta = std::make_shared<ProgressivePointCloud::Meta>();

    meta->set_timestamp(utils::GetNow());

    std::string message_id = utils::GenerateUUID();
    meta->set_frame_id(message_id);

    meta->set_height(msg->height);
    meta->set_width(msg->width);

    meta->set_is_bigendian(msg->is_bigendian);
    meta->set_point_step(msg->point_step);
    meta->set_row_step(msg->row_step);
    meta->set_is_dense(msg->is_dense);

    meta->set_min_x(config_->min_x);
    meta->set_max_x(config_->max_x);
    meta->set_min_y(config_->min_y);
    meta->set_max_y(config_->max_y);
    meta->set_min_z(config_->min_z);
    meta->set_max_z(config_->max_z);

    const size_t total_points = msg->width * msg->height;
    const size_t expected_chunk_num = total_points / config_->max_points_per_chunk;
    meta->set_expected_chunk_num(expected_chunk_num);

    for (const auto &ros_field: msg->fields) {
        auto *proto_field = meta->add_fields();
        proto_field->set_name(ros_field.name);
        proto_field->set_offset(ros_field.offset);
        proto_field->set_datatype(ros_field.datatype);
        proto_field->set_count(ros_field.count);
    }

    return meta;
}

bool ProgressivePointCloud2Sender::FindXYZOffsets(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    bool found_x = false, found_y = false, found_z = false;

    for (const auto &field: msg->fields) {
        if (field.name == "x") {
            config_->x_offset = field.offset;
            found_x = true;
        } else if (field.name == "y") {
            config_->y_offset = field.offset;
            found_y = true;
        } else if (field.name == "z") {
            config_->z_offset = field.offset;
            found_z = true;
        }
    }

    return found_x && found_y && found_z;
}

void ProgressivePointCloud2Sender::Initialize(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    // Find XYZ offsets
    if (!FindXYZOffsets(msg)) {
        throw std::runtime_error("Failed to find XYZ fields in PointCloud2 message");
    }

    // Compute bounding box
    ComputeBoundingBox(msg);

    // Calculate chunk size
    config_->point_step = msg->point_step;
    config_->total_points = msg->width * msg->height;

    // 240KB = 245760 bytes (considering safety margin from 256KB)
    constexpr size_t MAX_CHUNK_SIZE_BYTES = 240 * 1024;

    // Calculate maximum points per chunk
    config_->max_points_per_chunk = MAX_CHUNK_SIZE_BYTES / config_->point_step;

    // Calculate estimated chunks per level
    // Divide total points into 5 levels and estimate required chunks per level
    // TODO: NUM_LEVELS and data channel size??
    constexpr size_t NUM_LEVELS = 5;
    const size_t avg_points_per_level = config_->total_points / NUM_LEVELS;
    config_->estimated_chunks_per_level =
            (avg_points_per_level + config_->max_points_per_chunk - 1) / config_->max_points_per_chunk;

    initialized_ = true;
}

void ProgressivePointCloud2Sender::ComputeBoundingBox(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) const {
    const uint32_t point_step = msg->point_step;
    const uint8_t *data_ptr = msg->data.data();
    const size_t total_points = msg->width * msg->height;

    // Initialize min/max values
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    // Iterate through all points to find min/max
    for (size_t i = 0; i < total_points; ++i) {
        const uint8_t *point_ptr = data_ptr + i * point_step;

        float x, y, z;
        std::memcpy(&x, point_ptr + config_->x_offset, sizeof(float));
        std::memcpy(&y, point_ptr + config_->y_offset, sizeof(float));
        std::memcpy(&z, point_ptr + config_->z_offset, sizeof(float));

        // Process only valid points
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
            min_z = std::min(min_z, z);
            max_z = std::max(max_z, z);
        }
    }

    // Add 10% margin
    constexpr float MARGIN = 0.1f; // 10% margin
    const float x_range = max_x - min_x;
    const float y_range = max_y - min_y;
    const float z_range = max_z - min_z;

    const float x_margin = x_range * MARGIN;
    const float y_margin = y_range * MARGIN;
    const float z_margin = z_range * MARGIN;

    // Save to config (with margin applied)
    config_->min_x = min_x - x_margin;
    config_->max_x = max_x + x_margin;
    config_->min_y = min_y - y_margin;
    config_->max_y = max_y + y_margin;
    config_->min_z = min_z - z_margin;
    config_->max_z = max_z + z_margin;

    // Robot position-based ranges (assuming robot is at origin, using values with margin applied)
    config_->forward = config_->max_x; // Forward (positive x-axis direction)
    config_->backward = -config_->min_x; // Backward (negative x-axis direction)
    config_->right = config_->max_y; // Right (positive y-axis direction)
    config_->left = -config_->min_y; // Left (negative y-axis direction)
    config_->up = config_->max_z; // Up (positive z-axis direction)
    config_->down = -config_->min_z; // Down (negative z-axis direction)
}

ProgressivePointCloud2Sender::OctreeNode *ProgressivePointCloud2Sender::FindLeafNode(
    OctreeNode *node, const float x, const float y, const float z) {
    // Return if it's a leaf node
    if (node->is_leaf()) {
        return node;
    }

    // Calculate center point
    const float mid_x = (node->min_x + node->max_x) * 0.5f;
    const float mid_y = (node->min_y + node->max_y) * 0.5f;
    const float mid_z = (node->min_z + node->max_z) * 0.5f;

    // Calculate octant index
    int octant = 0;
    if (x >= mid_x)
        octant |= 1;
    if (y >= mid_y)
        octant |= 2;
    if (z >= mid_z)
        octant |= 4;

    // Recursively search child nodes
    return FindLeafNode(node->children[octant].get(), x, y, z);
}

void ProgressivePointCloud2Sender::CollectLeafNodes(OctreeNode *node, std::vector<OctreeNode *> &leaf_nodes) {
    if (node->is_leaf()) {
        // Collect only leaf nodes (if they have points)
        if (!node->point_indices.empty()) {
            leaf_nodes.push_back(node);
        }
    } else {
        // Recursively traverse child nodes
        for (const auto &child: node->children) {
            if (child) {
                CollectLeafNodes(child.get(), leaf_nodes);
            }
        }
    }
}

std::unique_ptr<ProgressivePointCloud2Sender::OctreeNode> ProgressivePointCloud2Sender::BuildOctree(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    // Step 1: Create root node (entire bounding box)
    auto root = std::make_unique<OctreeNode>();
    root->min_x = config_->min_x;
    root->max_x = config_->max_x;
    root->min_y = config_->min_y;
    root->max_y = config_->max_y;
    root->min_z = config_->min_z;
    root->max_z = config_->max_z;

    // Step 2: Read all valid points once
    const uint32_t point_step = msg->point_step;
    const uint8_t *data_ptr = msg->data.data();
    const size_t total_points = msg->width * msg->height;

    std::vector<PointData> points;
    points.reserve(total_points);

    for (size_t i = 0; i < total_points; ++i) {
        const uint8_t *point_ptr = data_ptr + i * point_step;

        PointData pd;
        std::memcpy(&pd.x, point_ptr + config_->x_offset, sizeof(float));
        std::memcpy(&pd.y, point_ptr + config_->y_offset, sizeof(float));
        std::memcpy(&pd.z, point_ptr + config_->z_offset, sizeof(float));

        if (std::isfinite(pd.x) && std::isfinite(pd.y) && std::isfinite(pd.z)) {
            pd.index = i;
            points.push_back(pd);
        }
    }

    // Step 3: Subdivide recursively using cached coordinates
    SubdivideNode(root.get(), points);

    return root;
}

void ProgressivePointCloud2Sender::SubdivideNode(OctreeNode *node, const std::vector<PointData> &points) {
    // Subdivision stopping condition 1: 10 or fewer points
    constexpr size_t MIN_POINTS_TO_SUBDIVIDE = 10;
    if (points.size() <= MIN_POINTS_TO_SUBDIVIDE) {
        node->point_indices.reserve(points.size());
        for (const auto &pd: points) {
            node->point_indices.push_back(pd.index);
        }
        return;
    }

    // Subdivision stopping condition 2: voxel size reached
    if (node->get_size() <= voxel_size_) {
        node->point_indices.reserve(points.size());
        for (const auto &pd: points) {
            node->point_indices.push_back(pd.index);
        }
        return;
    }

    // Calculate center point
    const float mid_x = (node->min_x + node->max_x) * 0.5f;
    const float mid_y = (node->min_y + node->max_y) * 0.5f;
    const float mid_z = (node->min_z + node->max_z) * 0.5f;

    // Create 8 child nodes and set spatial ranges
    for (int i = 0; i < 8; ++i) {
        node->children[i] = std::make_unique<OctreeNode>();
        const auto &child = node->children[i];

        // i's bits: [z][y][x]
        child->min_x = (i & 1) ? mid_x : node->min_x;
        child->max_x = (i & 1) ? node->max_x : mid_x;
        child->min_y = (i & 2) ? mid_y : node->min_y;
        child->max_y = (i & 2) ? node->max_y : mid_y;
        child->min_z = (i & 4) ? mid_z : node->min_z;
        child->max_z = (i & 4) ? node->max_z : mid_z;
    }

    // Classify points by octant using cached coordinates
    std::array<std::vector<PointData>, 8> octant_points;

    for (const auto &pd: points) {
        const int octant = GetOctant(pd.x, pd.y, pd.z, mid_x, mid_y, mid_z);
        octant_points[octant].push_back(pd);
    }

    // Recursively subdivide each child node
    for (int i = 0; i < 8; ++i) {
        if (!octant_points[i].empty()) {
            SubdivideNode(node->children[i].get(), octant_points[i]);
        }
    }
}

int ProgressivePointCloud2Sender::GetOctant(const float x,
                                            const float y,
                                            const float z,
                                            const float mid_x,
                                            const float mid_y,
                                            const float mid_z) {
    int octant = 0;
    if (x >= mid_x)
        octant |= 1;
    if (y >= mid_y)
        octant |= 2;
    if (z >= mid_z)
        octant |= 4;
    return octant;
}
