//
// Created by yhkim on 3/9/26.
//

#include "mosaic-ros2-sensor/util/simple_point_cloud2_sender.hpp"
#include "protobuf/pcc_chunk.pb.h"

#include "../util/clock_utils.hpp"
#include "../util/uuid_utils.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include <mosaic/handlers/data_channel/data_channel_handler.hpp>

using namespace mosaic::ros2::sensor_connector;

void SimplePointCloud2Sender::ProcessMsg(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    if (!initialized_) {
        config_ = std::make_unique<PointCloudConfig>();
        Initialize(msg);
    }

    const long created_timestamp = utils::GetNow();

    // Create and send metadata first
    const auto ppc_meta = ExtractMeta(msg);
    const std::string frame_id = ppc_meta->frame_id();
    SendData(ppc_meta->SerializeAsString());

    SendPoints(msg, frame_id, created_timestamp);
}

void SimplePointCloud2Sender::SendPoints(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                                         const std::string &frame_id, const long timestamp) {
    const uint32_t point_step = msg->point_step;
    const uint8_t *data_ptr = msg->data.data();
    const size_t total_points = msg->width * msg->height;

    // Filter out NaN/inf points
    std::vector<size_t> valid_indices;
    valid_indices.reserve(total_points);
    for (size_t i = 0; i < total_points; ++i) {
        float x, y, z;
        std::memcpy(&x, data_ptr + i * point_step + config_->x_offset, sizeof(float));
        std::memcpy(&y, data_ptr + i * point_step + config_->y_offset, sizeof(float));
        std::memcpy(&z, data_ptr + i * point_step + config_->z_offset, sizeof(float));
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            valid_indices.push_back(i);
        }
    }

    size_t chunk_idx = 0;
    size_t offset = 0;

    while (offset < valid_indices.size()) {
        const std::vector<uint8_t> chunk_buffer = BuildChunkBuffer(data_ptr, valid_indices, &offset, point_step);
        BuildChunkAndSend(chunk_buffer, frame_id, &chunk_idx, timestamp, point_step);
    }
}

std::vector<uint8_t> SimplePointCloud2Sender::BuildChunkBuffer(const uint8_t *data_ptr,
                                                               const std::vector<size_t> &valid_indices,
                                                               size_t *offset,
                                                               const uint32_t point_step) const {
    const size_t points_in_chunk = std::min(config_->max_points_per_chunk, valid_indices.size() - *offset);
    std::vector<uint8_t> chunk_buffer;
    chunk_buffer.reserve(points_in_chunk * point_step);
    for (size_t i = 0; i < points_in_chunk; ++i) {
        const uint8_t *point_ptr = data_ptr + valid_indices[*offset + i] * point_step;
        chunk_buffer.insert(chunk_buffer.end(), point_ptr, point_ptr + point_step);
    }
    *offset += points_in_chunk;
    return chunk_buffer;
}

void SimplePointCloud2Sender::BuildChunkAndSend(std::vector<uint8_t> chunk_buffer, const std::string &frame_id,
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

std::shared_ptr<ProgressivePointCloud::Meta> SimplePointCloud2Sender::ExtractMeta(
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

bool SimplePointCloud2Sender::FindXYZOffsets(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
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

void SimplePointCloud2Sender::Initialize(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
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

    initialized_ = true;
}

void SimplePointCloud2Sender::ComputeBoundingBox(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) const {
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

