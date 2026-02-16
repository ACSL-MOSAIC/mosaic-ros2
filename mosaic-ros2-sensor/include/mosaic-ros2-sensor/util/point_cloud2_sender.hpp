//
// Created by yhkim on 1/19/26.
//

#ifndef MOSAIC_ROS2_SENSOR_POINT_CLOUD_2_SENDER_HPP
#define MOSAIC_ROS2_SENSOR_POINT_CLOUD_2_SENDER_HPP

#include "mosaic-ros2-sensor/connector/point_cloud2_connector.hpp"
#include "protobuf/pc2_chunk.pb.h"
#include "protobuf/pc2_meta.pb.h"
#include "protobuf/pc2_statistic.pb.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mosaic::ros2::sensor_connector {
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

    struct LiDARStatisticMessage {
        std::string frame_id;

        // Timing information (in microseconds)
        long frame_received_timestamp; // Timestamp when frame is received
        long metadata_sent_timestamp; // Timestamp after metadata sent
        long octree_root_node_built_timestamp;
        long octree_subdivide_timestamp;
        long octree_built_timestamp; // Timestamp after octree built
        long all_chunks_sent_timestamp; // Timestamp after all chunks sent

        // Per-chunk timing (time after each chunk sent)
        std::vector<long> chunk_sent_timestamps; // Timestamp when each chunk was sent
    };

    class PointCloud2Sender {
    public:
        explicit PointCloud2Sender(float voxel_size);

        void AddPointCloud2DataChannel(const std::shared_ptr<PointCloud2DataChannel> &channel);

        void Send(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    private:
        bool IsAllChannelReady() const;

        std::shared_ptr<PointCloud2DataChannel> GetNextChannel();

        std::shared_ptr<point_cloud_2::Meta> ExtractMeta(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) const;

        void Initialize(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

        bool FindXYZOffsets(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                            unsigned int &x_offset,
                            unsigned int &y_offset,
                            unsigned int &z_offset) const;

        void ComputeBoundingBox(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

        OctreeNode *FindLeafNode(OctreeNode *node, float x, float y, float z);

        void CollectLeafNodes(OctreeNode *node, std::vector<OctreeNode *> &leaf_nodes);

        std::unique_ptr<OctreeNode> BuildOctree(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                                                const std::shared_ptr<LiDARStatisticMessage> &statistic);

        void SubdivideNode(OctreeNode *node,
                           const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                           const std::vector<size_t> &point_indices);

        int GetOctant(float x, float y, float z, float mid_x, float mid_y, float mid_z) const;

        void SendStatistic(const std::shared_ptr<LiDARStatisticMessage> &statistic);

        void SendData(const std::string &data);

        bool initialized_ = false;
        std::mutex send_mutex_;

        PointCloudConfig config_;

        float voxel_size_; // Minimum voxel size for octree leaf nodes (m)

        int channel_idx_ = 0;
        int max_channel_idx_;

        std::vector<std::shared_ptr<PointCloud2DataChannel> > point_cloud_2_data_channels_;
    };
} // namespace mosaic::ros2::sensor_connector

#endif  // MOSAIC_ROS2_SENSOR_POINT_CLOUD_2_SENDER_HPP
