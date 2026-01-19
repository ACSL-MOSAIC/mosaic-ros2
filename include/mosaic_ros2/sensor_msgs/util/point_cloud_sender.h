//
// Created by yhkim on 1/19/26.
//

#ifndef MOSAIC_ROS2_POINT_CLOUD_2_SENDER_H
#define MOSAIC_ROS2_POINT_CLOUD_2_SENDER_H

#include <utility>

#include "mosaic_ros2/sensor_msgs/point_cloud2_connector.h"
#include "pc2_chunk.pb.h"
#include "pc2_meta.pb.h"
#include "pc2_statistic.pb.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mosaic::ros2::sensor_connector {
// Octree 노드 구조
struct OctreeNode {
    // 공간 범위
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;

    // 자식 노드 (8개)
    std::unique_ptr<OctreeNode> children[8];

    // Leaf node인 경우 포인트 인덱스 저장
    std::vector<size_t> point_indices;

    // Leaf node 여부 확인
    bool is_leaf() const {
        return children[0] == nullptr;
    }

    // 공간 크기 계산
    float get_size() const {
        return std::max({max_x - min_x, max_y - min_y, max_z - min_z});
    }
};

struct PointCloudConfig {
    // XYZ field offsets
    unsigned int x_offset;
    unsigned int y_offset;
    unsigned int z_offset;

    // Bounding box (로봇 위치 기준)
    float forward;   // 앞쪽 최대 거리
    float backward;  // 뒤쪽 최대 거리
    float left;      // 왼쪽 최대 거리
    float right;     // 오른쪽 최대 거리
    float up;        // 위쪽 최대 거리
    float down;      // 아래쪽 최대 거리

    // 실제 min/max bounds
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;

    // Chunk 계산 정보
    uint32_t point_step;                // 한 포인트의 바이트 크기
    size_t max_points_per_chunk;        // 청크당 최대 포인트 개수
    size_t total_points;                // 전체 포인트 개수
    size_t estimated_chunks_per_level;  // 레벨당 예상 청크 개수
};

struct LiDARStatisticMessage {
    std::string frame_id;

    // Timing information (in microseconds)
    long frame_received_timestamp;  // 프레임을 받자마자 시간
    long metadata_sent_timestamp;   // metadata 전송한 후 시간
    long octree_root_node_built_timestamp;
    long octree_subdivide_timestamp;
    long octree_built_timestamp;     // octree 구성 후 시간
    long all_chunks_sent_timestamp;  // 모든 청크를 보내고 나서 시간

    // Per-chunk timing (각 청크 전송 후 시간)
    std::vector<long> chunk_sent_timestamps;  // 각 청크가 전송된 시간
};

class PointCloud2Sender {
  public:
    explicit PointCloud2Sender(float voxel_size);

    void AddPointCloud2DataChannel(const std::shared_ptr<PointCloud2DataChannel>& channel);

    void Send(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

  private:
    bool IsAllChannelReady() const;

    std::shared_ptr<PointCloud2DataChannel> GetNextChannel();

    std::shared_ptr<point_cloud_2::Meta> ExtractMeta(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) const;

    void Initialize(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

    bool FindXYZOffsets(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                        unsigned int& x_offset,
                        unsigned int& y_offset,
                        unsigned int& z_offset) const;

    void ComputeBoundingBox(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

    OctreeNode* FindLeafNode(OctreeNode* node, float x, float y, float z);

    void CollectLeafNodes(OctreeNode* node, std::vector<OctreeNode*>& leaf_nodes);

    std::unique_ptr<OctreeNode> BuildOctree(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                            LiDARStatisticMessage& statistic);

    void SubdivideNode(OctreeNode* node,
                       const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                       const std::vector<size_t>& point_indices);

    int GetOctant(float x, float y, float z, float mid_x, float mid_y, float mid_z) const;

    void SendStatistic(const LiDARStatisticMessage& statistic);

    void SendData(const std::string& data);

    bool initialized_ = false;
    std::mutex send_mutex_;

    PointCloudConfig config_;

    float voxel_size_;  // Octree leaf 노드의 최소 복셀 크기 (m)

    int channel_idx_ = 0;
    int max_channel_idx_;

    std::vector<std::shared_ptr<PointCloud2DataChannel>> point_cloud_2_data_channels_;
};
}  // namespace mosaic::ros2::sensor_connector

#endif  // MOSAIC_ROS2_POINT_CLOUD_2_SENDER_H
