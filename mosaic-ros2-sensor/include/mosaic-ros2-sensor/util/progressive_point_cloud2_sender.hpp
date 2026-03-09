//
// Created by yhkim on 1/19/26.
//

#ifndef MOSAIC_ROS2_SENSOR_POINT_CLOUD_2_SENDER_HPP
#define MOSAIC_ROS2_SENSOR_POINT_CLOUD_2_SENDER_HPP

#include "mosaic-ros2-sensor/connector/point_cloud2_connector.hpp"
#include "protobuf/pcc_meta.pb.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mosaic::ros2::sensor_connector {
    class ProgressivePointCloud2Sender : public PointCloud2Sender {
    public:
        ProgressivePointCloud2Sender() = default;

        ~ProgressivePointCloud2Sender() override = default;

        void SetParams(const std::unordered_map<std::string, std::string> &params) override;

        void ProcessAsync(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) override;

    private:
        struct OctreeNode;

        void SendInternal(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

        std::shared_ptr<ProgressivePointCloud::Meta> ExtractMeta(
            const sensor_msgs::msg::PointCloud2::SharedPtr &msg) const;

        void Initialize(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

        bool FindXYZOffsets(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

        void ComputeBoundingBox(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) const;

        OctreeNode *FindLeafNode(OctreeNode *node, float x, float y, float z);

        void CollectLeafNodes(OctreeNode *node, std::vector<OctreeNode *> &leaf_nodes);

        std::unique_ptr<OctreeNode> BuildOctree(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

        struct PointData {
            size_t index;
            float x, y, z;
        };

        void SubdivideNode(OctreeNode *node, const std::vector<PointData> &points);

        static int GetOctant(float x, float y, float z, float mid_x, float mid_y, float mid_z);

        void SendPoints(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                        const std::vector<OctreeNode *> &leaf_nodes, const std::string &frame_id, long timestamp);

        std::vector<uint8_t> BuildChunkBuffer(const std::vector<OctreeNode *> &leaf_nodes, const uint8_t *data_ptr,
                                              size_t *total_points_sent, bool *has_remaining_points,
                                              uint32_t point_step) const;

        void BuildChunkAndSend(std::vector<uint8_t> chunk_buffer, const std::string &frame_id, size_t *chunk_idx,
                               long timestamp, uint32_t point_step);

        // Octree node structure
        struct OctreeNode {
            // Spatial range
            float min_x, max_x;
            float min_y, max_y;
            float min_z, max_z;

            // Child nodes (8 children)
            std::unique_ptr<OctreeNode> children[8];

            // Store point indices for leaf nodes
            std::vector<size_t> point_indices;

            // Check if it's a leaf node
            bool is_leaf() const {
                return children[0] == nullptr;
            }

            // Calculate spatial size
            float get_size() const {
                return std::max({max_x - min_x, max_y - min_y, max_z - min_z});
            }
        };

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

        float voxel_size_ = 0.5; // Minimum voxel size for octree leaf nodes (m)
    };
} // namespace mosaic::ros2::sensor_connector

#endif  // MOSAIC_ROS2_SENSOR_POINT_CLOUD_2_SENDER_HPP
