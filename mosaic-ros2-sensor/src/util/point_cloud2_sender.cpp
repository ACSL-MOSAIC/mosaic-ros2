//
// Created by yhkim on 1/19/26.
//

#include "mosaic-ros2-sensor/util/point_cloud2_sender.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

#include <mosaic/handlers/data_channel/a_data_channel_handler.h>
#include <uuid/uuid.h>

using namespace mosaic::ros2::sensor_connector;

PointCloud2Sender::PointCloud2Sender(float voxel_size) : voxel_size_(voxel_size), max_channel_idx_(0) {}

void PointCloud2Sender::AddPointCloud2DataChannel(const std::shared_ptr<PointCloud2DataChannel>& channel) {
    point_cloud_2_data_channels_.push_back(channel);
    max_channel_idx_++;
}

long GetNow() {
    const auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
}

void PointCloud2Sender::Send(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    // Initialize the first frame
    if (!initialized_) {
        Initialize(msg);
    }

    if (!IsAllChannelReady()) {
        return;
    }

    // mutex lock
    // TODO: Consider asynchronous processing to avoid blocking when mutex lock is applied
    std::lock_guard lock(send_mutex_);

    // Prepare statistics message
    const auto statistic = std::make_shared<LiDARStatisticMessage>();

    // 1. Timestamp when frame is received
    statistic->frame_received_timestamp = GetNow();
    const long created_timestamp = statistic->frame_received_timestamp;

    // Create and send metadata first
    const auto ppc_meta = ExtractMeta(msg);
    const std::string frame_id = ppc_meta->frame_id();
    statistic->frame_id = frame_id;
    SendData(ppc_meta->SerializeAsString());

    // 2. Timestamp after sending metadata
    statistic->metadata_sent_timestamp = GetNow();

    // Build octree
    const auto octree_root = BuildOctree(msg, statistic);

    // Collect leaf nodes
    std::vector<OctreeNode*> leaf_nodes;
    CollectLeafNodes(octree_root.get(), leaf_nodes);

    // 3. Timestamp after building octree
    statistic->octree_built_timestamp = GetNow();

    // TODO: Extract logic below into a function

    const uint32_t point_step = msg->point_step;
    const uint8_t* data_ptr = msg->data.data();

    size_t chunk_idx = 0;
    size_t total_points_sent = 0;
    const size_t total_points = msg->width * msg->height;

    // Loop until all points are sent
    while (total_points_sent < total_points) {
        std::vector<uint8_t> chunk_buffer;
        chunk_buffer.reserve(config_.max_points_per_chunk * point_step);
        size_t points_in_chunk = 0;

        // Collect points from leaf nodes until chunk is full
        bool has_remaining_points = true;
        while (points_in_chunk < config_.max_points_per_chunk && has_remaining_points) {
            has_remaining_points = false;

            // Iterate through each leaf node
            for (auto* leaf : leaf_nodes) {
                // If this leaf still has points
                if (!leaf->point_indices.empty()) {
                    has_remaining_points = true;

                    // Extract one point from the back (pop_back is efficient)
                    const size_t point_idx = leaf->point_indices.back();
                    leaf->point_indices.pop_back();

                    // Add point data to chunk buffer
                    const uint8_t* point_ptr = data_ptr + point_idx * point_step;
                    chunk_buffer.insert(chunk_buffer.end(), point_ptr, point_ptr + point_step);

                    ++points_in_chunk;
                    ++total_points_sent;

                    // Stop if chunk is full
                    if (points_in_chunk >= config_.max_points_per_chunk) {
                        break;
                    }
                }
            }
        }

        // Send chunk if not empty
        if (!chunk_buffer.empty()) {
            const size_t chunk_point_count = chunk_buffer.size() / point_step;

            point_cloud_2::Chunk chunk;
            chunk.set_timestamp(created_timestamp);
            chunk.set_frame_id(frame_id);
            chunk.set_chunk_index(chunk_idx);
            chunk.set_point_size(chunk_point_count);
            chunk.set_data(chunk_buffer.data(), chunk_buffer.size());

            SendData(chunk.SerializeAsString());

            // Measure time after chunk sent
            statistic->chunk_sent_timestamps.push_back(GetNow());

            ++chunk_idx;
        }

        if (!has_remaining_points) {
            break;
        }
    }

    // 4. Timestamp after sending all chunks
    statistic->all_chunks_sent_timestamp = GetNow();

    // Send statistics data
    SendStatistic(statistic);
}

bool PointCloud2Sender::IsAllChannelReady() const {
    if (point_cloud_2_data_channels_.empty()) {
        return false;
    }
    for (const auto& channel : point_cloud_2_data_channels_) {
        if (channel->Sendable()) {
            return true;
        }
    }
    return false;
}

std::shared_ptr<PointCloud2DataChannel> PointCloud2Sender::GetNextChannel() {
    channel_idx_++;
    channel_idx_ %= max_channel_idx_;
    if (point_cloud_2_data_channels_[channel_idx_]->GetState() == handlers::ADataChannelHandler::kUnknown) {
        return nullptr;
    }
    if (point_cloud_2_data_channels_[channel_idx_]->Sendable()) {
        return point_cloud_2_data_channels_[channel_idx_];
    }
    // 0.1 ms sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return GetNextChannel();
}

std::string GenerateUUID() {
    uuid_t uuid;
    uuid_generate_random(uuid);
    char uuid_str[37];
    uuid_unparse_lower(uuid, uuid_str);
    return std::string(uuid_str);
}

std::shared_ptr<point_cloud_2::Meta> PointCloud2Sender::ExtractMeta(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) const {
    auto meta = std::make_shared<point_cloud_2::Meta>();

    meta->set_timestamp(GetNow());

    std::string message_id = GenerateUUID();
    meta->set_frame_id(message_id);

    meta->set_height(msg->height);
    meta->set_width(msg->width);

    meta->set_is_bigendian(msg->is_bigendian);
    meta->set_point_step(msg->point_step);
    meta->set_row_step(msg->row_step);
    meta->set_is_dense(msg->is_dense);

    meta->set_min_x(config_.min_x);
    meta->set_max_x(config_.max_x);
    meta->set_min_y(config_.min_y);
    meta->set_max_y(config_.max_y);
    meta->set_min_z(config_.min_z);
    meta->set_max_z(config_.max_z);

    for (const auto& ros_field : msg->fields) {
        auto* proto_field = meta->add_fields();
        proto_field->set_name(ros_field.name);
        proto_field->set_offset(ros_field.offset);
        proto_field->set_datatype(ros_field.datatype);
        proto_field->set_count(ros_field.count);
    }

    return meta;
}

bool PointCloud2Sender::FindXYZOffsets(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                       unsigned int& x_offset,
                                       unsigned int& y_offset,
                                       unsigned int& z_offset) const {
    bool found_x = false, found_y = false, found_z = false;

    for (const auto& field : msg->fields) {
        if (field.name == "x") {
            x_offset = field.offset;
            found_x = true;
        } else if (field.name == "y") {
            y_offset = field.offset;
            found_y = true;
        } else if (field.name == "z") {
            z_offset = field.offset;
            found_z = true;
        }
    }

    return found_x && found_y && found_z;
}

void PointCloud2Sender::Initialize(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    // Find XYZ offsets
    if (!FindXYZOffsets(msg, config_.x_offset, config_.y_offset, config_.z_offset)) {
        throw std::runtime_error("Failed to find XYZ fields in PointCloud2 message");
    }

    // Compute bounding box
    ComputeBoundingBox(msg);

    // Calculate chunk size
    config_.point_step = msg->point_step;
    config_.total_points = msg->width * msg->height;

    // 240KB = 245760 bytes (considering safety margin from 256KB)
    constexpr size_t MAX_CHUNK_SIZE_BYTES = 240 * 1024;

    // Calculate maximum points per chunk
    config_.max_points_per_chunk = MAX_CHUNK_SIZE_BYTES / config_.point_step;

    // Calculate estimated chunks per level
    // Divide total points into 5 levels and estimate required chunks per level
    // TODO: NUM_LEVELS and data channel size??
    constexpr size_t NUM_LEVELS = 5;
    const size_t avg_points_per_level = config_.total_points / NUM_LEVELS;
    config_.estimated_chunks_per_level =
        (avg_points_per_level + config_.max_points_per_chunk - 1) / config_.max_points_per_chunk;

    initialized_ = true;
}

void PointCloud2Sender::ComputeBoundingBox(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    const uint32_t point_step = msg->point_step;
    const uint8_t* data_ptr = msg->data.data();
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
        const uint8_t* point_ptr = data_ptr + i * point_step;

        float x, y, z;
        std::memcpy(&x, point_ptr + config_.x_offset, sizeof(float));
        std::memcpy(&y, point_ptr + config_.y_offset, sizeof(float));
        std::memcpy(&z, point_ptr + config_.z_offset, sizeof(float));

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
    constexpr float MARGIN = 0.1f;  // 10% margin
    const float x_range = max_x - min_x;
    const float y_range = max_y - min_y;
    const float z_range = max_z - min_z;

    const float x_margin = x_range * MARGIN;
    const float y_margin = y_range * MARGIN;
    const float z_margin = z_range * MARGIN;

    // Save to config (with margin applied)
    config_.min_x = min_x - x_margin;
    config_.max_x = max_x + x_margin;
    config_.min_y = min_y - y_margin;
    config_.max_y = max_y + y_margin;
    config_.min_z = min_z - z_margin;
    config_.max_z = max_z + z_margin;

    // Robot position-based ranges (assuming robot is at origin, using values with margin applied)
    config_.forward = config_.max_x;    // Forward (positive x-axis direction)
    config_.backward = -config_.min_x;  // Backward (negative x-axis direction)
    config_.right = config_.max_y;      // Right (positive y-axis direction)
    config_.left = -config_.min_y;      // Left (negative y-axis direction)
    config_.up = config_.max_z;         // Up (positive z-axis direction)
    config_.down = -config_.min_z;      // Down (negative z-axis direction)
}

void PointCloud2Sender::SendData(const std::string& data) {
    if (const auto channel = GetNextChannel(); channel != nullptr) {
        channel->SendStringAsByte(data, false);
    }
}

OctreeNode* PointCloud2Sender::FindLeafNode(OctreeNode* node, const float x, const float y, const float z) {
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

void PointCloud2Sender::CollectLeafNodes(OctreeNode* node, std::vector<OctreeNode*>& leaf_nodes) {
    if (node->is_leaf()) {
        // Collect only leaf nodes (if they have points)
        if (!node->point_indices.empty()) {
            leaf_nodes.push_back(node);
        }
    } else {
        // Recursively traverse child nodes
        for (const auto& child : node->children) {
            if (child) {
                CollectLeafNodes(child.get(), leaf_nodes);
            }
        }
    }
}

std::unique_ptr<OctreeNode> PointCloud2Sender::BuildOctree(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                                           const std::shared_ptr<LiDARStatisticMessage>& statistic) {
    // Step 1: Create root node (entire bounding box)
    auto root = std::make_unique<OctreeNode>();
    root->min_x = config_.min_x;
    root->max_x = config_.max_x;
    root->min_y = config_.min_y;
    root->max_y = config_.max_y;
    root->min_z = config_.min_z;
    root->max_z = config_.max_z;

    // Step 2: Collect all valid points and assign to root
    const uint32_t point_step = msg->point_step;
    const uint8_t* data_ptr = msg->data.data();
    const size_t total_points = msg->width * msg->height;

    std::vector<size_t> root_point_indices;
    root_point_indices.reserve(total_points);

    for (size_t point_idx = 0; point_idx < total_points; ++point_idx) {
        const uint8_t* point_ptr = data_ptr + point_idx * point_step;

        // Read XYZ coordinates
        float x, y, z;
        std::memcpy(&x, point_ptr + config_.x_offset, sizeof(float));
        std::memcpy(&y, point_ptr + config_.y_offset, sizeof(float));
        std::memcpy(&z, point_ptr + config_.z_offset, sizeof(float));

        // Process only valid points
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            root_point_indices.push_back(point_idx);
        }
    }

    // Time after root node creation
    statistic->octree_root_node_built_timestamp = GetNow();

    // Step 3: Subdivide recursively and assign points
    SubdivideNode(root.get(), msg, root_point_indices);

    // Time after spatial subdivision
    statistic->octree_subdivide_timestamp = GetNow();

    return root;
}

void PointCloud2Sender::SubdivideNode(OctreeNode* node,
                                      const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                      const std::vector<size_t>& point_indices) {
    // Subdivision stopping condition 1: 10 or fewer points
    constexpr size_t MIN_POINTS_TO_SUBDIVIDE = 10;
    if (point_indices.size() <= MIN_POINTS_TO_SUBDIVIDE) {
        node->point_indices = point_indices;
        return;
    }

    // Subdivision stopping condition 2: voxel size reached
    if (node->get_size() <= voxel_size_) {
        node->point_indices = point_indices;
        return;
    }

    // Calculate center point
    const float mid_x = (node->min_x + node->max_x) * 0.5f;
    const float mid_y = (node->min_y + node->max_y) * 0.5f;
    const float mid_z = (node->min_z + node->max_z) * 0.5f;

    // Create 8 child nodes and set spatial ranges
    for (int i = 0; i < 8; ++i) {
        node->children[i] = std::make_unique<OctreeNode>();
        const auto& child = node->children[i];

        // Set spatial range according to octant index
        // i's bits: [z][y][x]
        child->min_x = (i & 1) ? mid_x : node->min_x;
        child->max_x = (i & 1) ? node->max_x : mid_x;
        child->min_y = (i & 2) ? mid_y : node->min_y;
        child->max_y = (i & 2) ? node->max_y : mid_y;
        child->min_z = (i & 4) ? mid_z : node->min_z;
        child->max_z = (i & 4) ? node->max_z : mid_z;
    }

    // Classify points by octant
    std::array<std::vector<size_t>, 8> octant_points;

    const uint32_t point_step = msg->point_step;
    const uint8_t* data_ptr = msg->data.data();

    for (size_t point_idx : point_indices) {
        const uint8_t* point_ptr = data_ptr + point_idx * point_step;

        // Read XYZ coordinates
        float x, y, z;
        std::memcpy(&x, point_ptr + config_.x_offset, sizeof(float));
        std::memcpy(&y, point_ptr + config_.y_offset, sizeof(float));
        std::memcpy(&z, point_ptr + config_.z_offset, sizeof(float));

        // Calculate which octant it belongs to
        const int octant = GetOctant(x, y, z, mid_x, mid_y, mid_z);
        octant_points[octant].push_back(point_idx);
    }

    // Recursively subdivide each child node
    for (int i = 0; i < 8; ++i) {
        if (!octant_points[i].empty()) {
            SubdivideNode(node->children[i].get(), msg, octant_points[i]);
        }
    }
}

int PointCloud2Sender::GetOctant(const float x,
                                 const float y,
                                 const float z,
                                 const float mid_x,
                                 const float mid_y,
                                 const float mid_z) const {
    int octant = 0;
    if (x >= mid_x)
        octant |= 1;
    if (y >= mid_y)
        octant |= 2;
    if (z >= mid_z)
        octant |= 4;
    return octant;
}

void PointCloud2Sender::SendStatistic(const std::shared_ptr<LiDARStatisticMessage>& statistic) {
    const auto ppcStatistic = std::make_shared<point_cloud_2::Statistic>();

    ppcStatistic->set_timestamp(GetNow());
    ppcStatistic->set_frame_id(statistic->frame_id);

    ppcStatistic->set_frame_received_timestamp(statistic->frame_received_timestamp);
    ppcStatistic->set_metadata_sent_timestamp(statistic->metadata_sent_timestamp);
    ppcStatistic->set_octree_root_node_built_timestamp(statistic->octree_root_node_built_timestamp);
    ppcStatistic->set_octree_subdivide_timestamp(statistic->octree_subdivide_timestamp);
    ppcStatistic->set_octree_built_timestamp(statistic->octree_built_timestamp);
    ppcStatistic->set_all_chunks_sent_timestamp(statistic->all_chunks_sent_timestamp);

    for (const auto& ts : statistic->chunk_sent_timestamps) {
        ppcStatistic->add_chunk_sent_timestamps(ts);
    }

    SendData(ppcStatistic->SerializeAsString());
}
