//
// Created by yhkim on 1/19/26.
//

#include "mosaic_ros2/sensor_msgs/util/point_cloud_sender.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

#include <uuid/uuid.h>

ProgressivePointCloudSender::ProgressivePointCloudSender(float voxel_size)
    : voxel_size_(voxel_size), max_channel_idx_(0) {}

void ProgressivePointCloudSender::AddLiDARDataChannels(const std::shared_ptr<husky::LiDARDataChannel>& channel) {
    lidar_data_channels_.push_back(channel);
    max_channel_idx_++;
}

long GetNow() {
    const auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
}

void ProgressivePointCloudSender::Send(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    // 최초 프레임 초기화
    if (!initialized_) {
        Initialize(msg);
    }

    if (!IsAllChannelReady()) {
        return;
    }

    // mutex lock
    // TODO: mutex lock 걸 시 블로킹이 될 수 있으므로 비동기로 처리하는 방법 고민 필요
    std::lock_guard lock(send_mutex_);

    // 통계 메시지 준비
    // TODO: make_shared
    LiDARStatisticMessage statistic;

    // 1. 프레임을 받자마자 시간
    statistic.frame_received_timestamp = GetNow();
    const long created_timestamp = statistic.frame_received_timestamp;

    // Metadata 먼저 생성 후 전달
    const auto ppc_meta = ExtractMeta(msg);
    const std::string frame_id = ppc_meta->frame_id();
    statistic.frame_id = frame_id;
    SendData(ppc_meta->SerializeAsString());

    // 2. Metadata 전송한 후 시간
    statistic.metadata_sent_timestamp = GetNow();

    // Octree 구성
    const auto octree_root = BuildOctree(msg, statistic);

    // Leaf 노드 수집
    std::vector<OctreeNode*> leaf_nodes;
    CollectLeafNodes(octree_root.get(), leaf_nodes);

    // 3. Octree 구성 후 시간
    statistic.octree_built_timestamp = GetNow();

    // TODO: 아래 로직 함수로 빼기

    const uint32_t point_step = msg->point_step;
    const uint8_t* data_ptr = msg->data.data();

    size_t chunk_idx = 0;
    size_t total_points_sent = 0;
    const size_t total_points = msg->width * msg->height;

    // 모든 포인트를 전송할 때까지 반복
    while (total_points_sent < total_points) {
        std::vector<uint8_t> chunk_buffer;
        chunk_buffer.reserve(config_.max_points_per_chunk * point_step);
        size_t points_in_chunk = 0;

        // 청크가 가득 찰 때까지 leaf 노드들을 순회하며 포인트 수집
        bool has_remaining_points = true;
        while (points_in_chunk < config_.max_points_per_chunk && has_remaining_points) {
            has_remaining_points = false;

            // 각 leaf 노드를 순회
            for (auto* leaf : leaf_nodes) {
                // 이 leaf에 아직 포인트가 있으면
                if (!leaf->point_indices.empty()) {
                    has_remaining_points = true;

                    // 맨 뒤에서 포인트를 하나 빼오기 (pop_back이 효율적)
                    const size_t point_idx = leaf->point_indices.back();
                    leaf->point_indices.pop_back();

                    // 포인트 데이터를 청크 버퍼에 추가
                    const uint8_t* point_ptr = data_ptr + point_idx * point_step;
                    chunk_buffer.insert(chunk_buffer.end(), point_ptr, point_ptr + point_step);

                    ++points_in_chunk;
                    ++total_points_sent;

                    // 청크가 가득 찼으면 중단
                    if (points_in_chunk >= config_.max_points_per_chunk) {
                        break;
                    }
                }
            }
        }

        // 청크가 비어있지 않으면 전송
        if (!chunk_buffer.empty()) {
            const size_t chunk_point_count = chunk_buffer.size() / point_step;

            ppc_chunk::PPCChunk chunk;
            chunk.set_timestamp(created_timestamp);
            chunk.set_frame_id(frame_id);
            chunk.set_chunk_index(chunk_idx);
            chunk.set_point_size(chunk_point_count);
            chunk.set_data(chunk_buffer.data(), chunk_buffer.size());

            SendData(chunk.SerializeAsString());

            // 청크 전송 후 시간 측정
            statistic.chunk_sent_timestamps.push_back(GetNow());

            ++chunk_idx;
        }

        if (!has_remaining_points) {
            break;
        }
    }

    // 4. 모든 청크를 보내고 나서 시간
    statistic.all_chunks_sent_timestamp = GetNow();

    // 통계 데이터 전송
    SendStatistic(statistic);
}

bool ProgressivePointCloudSender::IsAllChannelReady() const {
    if (lidar_data_channels_.empty()) {
        return false;
    }
    for (const auto& channel : lidar_data_channels_) {
        if (channel->Sendable()) {
            return true;
        }
    }
    return false;
}

std::shared_ptr<husky::LiDARDataChannel> ProgressivePointCloudSender::GetNextChannel() {
    channel_idx_++;
    channel_idx_ %= max_channel_idx_;
    if (lidar_data_channels_[channel_idx_]->GetState() == ADataChannelHandler::kUnknown) {
        return nullptr;
    }
    if (lidar_data_channels_[channel_idx_]->Sendable()) {
        return lidar_data_channels_[channel_idx_];
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

std::shared_ptr<ppc_meta::PPCMeta> ProgressivePointCloudSender::ExtractMeta(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) const {
    auto meta = std::make_shared<ppc_meta::PPCMeta>();

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

bool ProgressivePointCloudSender::FindXYZOffsets(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
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

void ProgressivePointCloudSender::Initialize(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    // XYZ offsets 찾기
    if (!FindXYZOffsets(msg, config_.x_offset, config_.y_offset, config_.z_offset)) {
        throw std::runtime_error("Failed to find XYZ fields in PointCloud2 message");
    }

    // Bounding box 계산
    ComputeBoundingBox(msg);

    // Chunk 크기 계산
    config_.point_step = msg->point_step;
    config_.total_points = msg->width * msg->height;

    // 240KB = 245760 bytes (256KB에서 안전 마진 고려)
    constexpr size_t MAX_CHUNK_SIZE_BYTES = 240 * 1024;

    // 청크당 최대 포인트 개수 계산
    config_.max_points_per_chunk = MAX_CHUNK_SIZE_BYTES / config_.point_step;

    // 레벨당 예상 청크 개수 계산
    // 전체 포인트를 5개 레벨로 나누고, 각 레벨당 필요한 청크 개수 추정
    constexpr size_t NUM_LEVELS = 5;
    const size_t avg_points_per_level = config_.total_points / NUM_LEVELS;
    config_.estimated_chunks_per_level =
        (avg_points_per_level + config_.max_points_per_chunk - 1) / config_.max_points_per_chunk;

    initialized_ = true;
}

void ProgressivePointCloudSender::ComputeBoundingBox(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    const uint32_t point_step = msg->point_step;
    const uint8_t* data_ptr = msg->data.data();
    const size_t total_points = msg->width * msg->height;

    // 초기값 설정
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    // 모든 포인트를 순회하며 min/max 찾기
    for (size_t i = 0; i < total_points; ++i) {
        const uint8_t* point_ptr = data_ptr + i * point_step;

        float x, y, z;
        std::memcpy(&x, point_ptr + config_.x_offset, sizeof(float));
        std::memcpy(&y, point_ptr + config_.y_offset, sizeof(float));
        std::memcpy(&z, point_ptr + config_.z_offset, sizeof(float));

        // 유효한 포인트만 처리
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
            min_z = std::min(min_z, z);
            max_z = std::max(max_z, z);
        }
    }

    // 10% 마진 추가
    constexpr float MARGIN = 0.1f;  // 10% 마진
    const float x_range = max_x - min_x;
    const float y_range = max_y - min_y;
    const float z_range = max_z - min_z;

    const float x_margin = x_range * MARGIN;
    const float y_margin = y_range * MARGIN;
    const float z_margin = z_range * MARGIN;

    // config에 저장 (마진 적용)
    config_.min_x = min_x - x_margin;
    config_.max_x = max_x + x_margin;
    config_.min_y = min_y - y_margin;
    config_.max_y = max_y + y_margin;
    config_.min_z = min_z - z_margin;
    config_.max_z = max_z + z_margin;

    // 로봇 위치 기준 범위 (로봇이 원점에 있다고 가정, 마진 적용된 값 사용)
    config_.forward = config_.max_x;    // 앞쪽 (x축 양의 방향)
    config_.backward = -config_.min_x;  // 뒤쪽 (x축 음의 방향)
    config_.right = config_.max_y;      // 오른쪽 (y축 양의 방향)
    config_.left = -config_.min_y;      // 왼쪽 (y축 음의 방향)
    config_.up = config_.max_z;         // 위쪽 (z축 양의 방향)
    config_.down = -config_.min_z;      // 아래쪽 (z축 음의 방향)
}

void ProgressivePointCloudSender::SendData(const std::string& data) {
    if (const auto channel = GetNextChannel(); channel != nullptr) {
        channel->SendStringAsByte(data, false);
    }
}

OctreeNode* ProgressivePointCloudSender::FindLeafNode(OctreeNode* node, const float x, const float y, const float z) {
    // Leaf 노드면 반환
    if (node->is_leaf()) {
        return node;
    }

    // 중심점 계산
    const float mid_x = (node->min_x + node->max_x) * 0.5f;
    const float mid_y = (node->min_y + node->max_y) * 0.5f;
    const float mid_z = (node->min_z + node->max_z) * 0.5f;

    // Octant 인덱스 계산
    int octant = 0;
    if (x >= mid_x)
        octant |= 1;
    if (y >= mid_y)
        octant |= 2;
    if (z >= mid_z)
        octant |= 4;

    // 재귀적으로 자식 노드 탐색
    return FindLeafNode(node->children[octant].get(), x, y, z);
}

void ProgressivePointCloudSender::CollectLeafNodes(OctreeNode* node, std::vector<OctreeNode*>& leaf_nodes) {
    if (node->is_leaf()) {
        // Leaf 노드만 수집 (포인트가 있는 경우)
        if (!node->point_indices.empty()) {
            leaf_nodes.push_back(node);
        }
    } else {
        // 자식 노드들을 재귀적으로 순회
        for (const auto& child : node->children) {
            if (child) {
                CollectLeafNodes(child.get(), leaf_nodes);
            }
        }
    }
}

std::unique_ptr<OctreeNode> ProgressivePointCloudSender::BuildOctree(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    LiDARStatisticMessage& statistic) {
    // 1단계: 루트 노드 생성 (전체 bounding box)
    auto root = std::make_unique<OctreeNode>();
    root->min_x = config_.min_x;
    root->max_x = config_.max_x;
    root->min_y = config_.min_y;
    root->max_y = config_.max_y;
    root->min_z = config_.min_z;
    root->max_z = config_.max_z;

    // 2단계: 모든 유효한 포인트를 수집하여 루트에 할당
    const uint32_t point_step = msg->point_step;
    const uint8_t* data_ptr = msg->data.data();
    const size_t total_points = msg->width * msg->height;

    std::vector<size_t> root_point_indices;
    root_point_indices.reserve(total_points);

    for (size_t point_idx = 0; point_idx < total_points; ++point_idx) {
        const uint8_t* point_ptr = data_ptr + point_idx * point_step;

        // XYZ 좌표 읽기
        float x, y, z;
        std::memcpy(&x, point_ptr + config_.x_offset, sizeof(float));
        std::memcpy(&y, point_ptr + config_.y_offset, sizeof(float));
        std::memcpy(&z, point_ptr + config_.z_offset, sizeof(float));

        // 유효한 포인트만 수집
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            root_point_indices.push_back(point_idx);
        }
    }

    // 루트 노드 생성 후 시간
    statistic.octree_root_node_built_timestamp = GetNow();

    // 3단계: 재귀적으로 분할하며 포인트 할당
    SubdivideNode(root.get(), msg, root_point_indices);

    // 공간 분할 후 시간
    statistic.octree_subdivide_timestamp = GetNow();

    return root;
}

void ProgressivePointCloudSender::SubdivideNode(OctreeNode* node,
                                                const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                                const std::vector<size_t>& point_indices) {
    // 분할 중단 조건 1: 포인트가 10개 이하
    constexpr size_t MIN_POINTS_TO_SUBDIVIDE = 10;
    if (point_indices.size() <= MIN_POINTS_TO_SUBDIVIDE) {
        node->point_indices = point_indices;
        return;
    }

    // 분할 중단 조건 2: voxel size에 도달
    if (node->get_size() <= voxel_size_) {
        node->point_indices = point_indices;
        return;
    }

    // 중심점 계산
    const float mid_x = (node->min_x + node->max_x) * 0.5f;
    const float mid_y = (node->min_y + node->max_y) * 0.5f;
    const float mid_z = (node->min_z + node->max_z) * 0.5f;

    // 8개의 자식 노드 생성 및 공간 범위 설정
    for (int i = 0; i < 8; ++i) {
        node->children[i] = std::make_unique<OctreeNode>();
        const auto& child = node->children[i];

        // Octant 인덱스에 따라 공간 범위 설정
        // i의 비트: [z][y][x]
        child->min_x = (i & 1) ? mid_x : node->min_x;
        child->max_x = (i & 1) ? node->max_x : mid_x;
        child->min_y = (i & 2) ? mid_y : node->min_y;
        child->max_y = (i & 2) ? node->max_y : mid_y;
        child->min_z = (i & 4) ? mid_z : node->min_z;
        child->max_z = (i & 4) ? node->max_z : mid_z;
    }

    // 각 octant별로 포인트를 분류
    std::array<std::vector<size_t>, 8> octant_points;

    const uint32_t point_step = msg->point_step;
    const uint8_t* data_ptr = msg->data.data();

    for (size_t point_idx : point_indices) {
        const uint8_t* point_ptr = data_ptr + point_idx * point_step;

        // XYZ 좌표 읽기
        float x, y, z;
        std::memcpy(&x, point_ptr + config_.x_offset, sizeof(float));
        std::memcpy(&y, point_ptr + config_.y_offset, sizeof(float));
        std::memcpy(&z, point_ptr + config_.z_offset, sizeof(float));

        // 어느 octant에 속하는지 계산
        const int octant = GetOctant(x, y, z, mid_x, mid_y, mid_z);
        octant_points[octant].push_back(point_idx);
    }

    // 각 자식 노드를 재귀적으로 분할
    for (int i = 0; i < 8; ++i) {
        if (!octant_points[i].empty()) {
            SubdivideNode(node->children[i].get(), msg, octant_points[i]);
        }
    }
}

int ProgressivePointCloudSender::GetOctant(const float x,
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

void ProgressivePointCloudSender::SendStatistic(const LiDARStatisticMessage& statistic) {
    const auto ppcStatistic = std::make_shared<ppc_statistic::PPCStatistic>();

    ppcStatistic->set_timestamp(GetNow());
    ppcStatistic->set_frame_id(statistic.frame_id);

    ppcStatistic->set_frame_received_timestamp(statistic.frame_received_timestamp);
    ppcStatistic->set_metadata_sent_timestamp(statistic.metadata_sent_timestamp);
    ppcStatistic->set_octree_root_node_built_timestamp(statistic.octree_root_node_built_timestamp);
    ppcStatistic->set_octree_subdivide_timestamp(statistic.octree_subdivide_timestamp);
    ppcStatistic->set_octree_built_timestamp(statistic.octree_built_timestamp);
    ppcStatistic->set_all_chunks_sent_timestamp(statistic.all_chunks_sent_timestamp);

    for (const auto& ts : statistic.chunk_sent_timestamps) {
        ppcStatistic->add_chunk_sent_timestamps(ts);
    }

    SendData(ppcStatistic->SerializeAsString());
}
