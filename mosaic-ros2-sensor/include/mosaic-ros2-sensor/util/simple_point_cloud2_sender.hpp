//
// Created by yhkim on 1/19/26.
//

#ifndef MOSAIC_ROS2_SENSOR_SIMPLE_POINT_CLOUD_2_SENDER_HPP
#define MOSAIC_ROS2_SENSOR_SIMPLE_POINT_CLOUD_2_SENDER_HPP

#include "mosaic-ros2-sensor/connector/point_cloud2_connector.hpp"
#include "protobuf/pcc_meta.pb.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mosaic::ros2::sensor_connector {
    class SimplePointCloud2Sender : public PointCloud2Sender {
    public:
        SimplePointCloud2Sender() = default;

        ~SimplePointCloud2Sender() override = default;

        void SetParams(const std::unordered_map<std::string, std::string> &) override {}

        void ProcessAsync(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) override;

    private:
        void SendInternal(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

        std::shared_ptr<ProgressivePointCloud::Meta> ExtractMeta(
            const sensor_msgs::msg::PointCloud2::SharedPtr &msg) const;

        void Initialize(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

        bool FindXYZOffsets(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

        void ComputeBoundingBox(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) const;

        void SendPoints(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                        const std::string &frame_id, long timestamp);

        std::vector<uint8_t> BuildChunkBuffer(const uint8_t *data_ptr,
                                              const std::vector<size_t> &valid_indices,
                                              size_t *offset, uint32_t point_step) const;

        void BuildChunkAndSend(std::vector<uint8_t> chunk_buffer, const std::string &frame_id, size_t *chunk_idx,
                               long timestamp, uint32_t point_step);

        struct PointCloudConfig {
            // XYZ field offsets
            unsigned int x_offset;
            unsigned int y_offset;
            unsigned int z_offset;

            // Bounding box (robot position-based)
            float forward; // Forward maximum distance
            float backward; // Backward maximum distance
            float left; // Left maximum distance
            float right; // Right maximum distance
            float up; // Up maximum distance
            float down; // Down maximum distance

            // Actual min/max bounds
            float min_x, max_x;
            float min_y, max_y;
            float min_z, max_z;

            // Chunk calculation information
            uint32_t point_step; // Byte size of one point
            size_t max_points_per_chunk; // Maximum points per chunk
            size_t total_points; // Total number of points
            size_t estimated_chunks_per_level; // Estimated chunks per level
        };

        bool initialized_ = false;
        bool try_sending = false;
        std::mutex mutex_;

        std::unique_ptr<PointCloudConfig> config_;
    };
} // namespace mosaic::ros2::sensor_connector

#endif  // MOSAIC_ROS2_SENSOR_SIMPLE_POINT_CLOUD_2_SENDER_HPP
