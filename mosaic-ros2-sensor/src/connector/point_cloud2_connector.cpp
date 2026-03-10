//
// Created by yhkim on 1/19/26.
//

#include "mosaic-ros2-sensor/connector/point_cloud2_connector.hpp"

#include <chrono>
#include <thread>

#include "mosaic-ros2-sensor/util/progressive_point_cloud2_sender.hpp"
#include "mosaic-ros2-sensor/util/simple_point_cloud2_sender.hpp"

using namespace mosaic::ros2::sensor_connector;

void PointCloud2ConnectorConfigurer::Configure() {
    MOSAIC_LOG_INFO("Configuring ROS2 sensor_msgs::PointCloud2 Connector...");

    if (const auto stream_mode = connector_config_->params.at("stream_mode"); stream_mode == "progressive") {
        point_cloud2_sender_ = std::make_unique<ProgressivePointCloud2Sender>();
    } else if (stream_mode == "simple") {
        point_cloud2_sender_ = std::make_unique<SimplePointCloud2Sender>();
    } else {
        MOSAIC_LOG_ERROR("Unsupported stream mode: {}, use simple mode alternatively.", stream_mode);
        point_cloud2_sender_ = std::make_unique<SimplePointCloud2Sender>();
    }

    // TODO: MIN_POINTS_TO_SUBDIVIDE to configurable params
    point_cloud2_sender_->SetParams(connector_config_->params);

    for (int i = 0; i < parallel_num_; i++) {
        const auto channel_label = connector_config_->label + "-" + std::to_string(i);
        const auto handler = std::make_shared<PointCloud2DataChannel>(channel_label, mosaic_node_);
        handlers_.push_back(handler);
        point_cloud2_sender_->AddPointCloud2DataChannel(handler);
    }

    const auto qos = rclcpp::QoS(1).best_effort().durability_volatile();

    subscription_ = mosaic_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name_, qos, std::bind(&PointCloud2ConnectorConfigurer::Callback, this, std::placeholders::_1));

    mosaic_node_->AddSubscription(subscription_);

    running_ = true;

    MOSAIC_LOG_INFO("Configuring ROS2 sensor_msgs::PointCloud2 Connector Done!");
}

void PointCloud2ConnectorConfigurer::Callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    MOSAIC_LOG_DEBUG("sensor_msgs::msg::PointCloud2 received");
    point_cloud2_sender_->ProcessAsync(msg);
}

void PointCloud2Sender::AddPointCloud2DataChannel(const std::shared_ptr<PointCloud2DataChannel> &channel) {
    data_channels_.push_back(channel);
    max_channel_idx_++;
    MOSAIC_LOG_INFO("Add data channel: {}", max_channel_idx_);
}

void PointCloud2Sender::ProcessAsync(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    {
        std::lock_guard lock(mutex_);
        if (sending_ == true) {
            MOSAIC_LOG_ERROR("Frame drop!");
            return;
        }

        if (!IsAllChannelReady()) {
            MOSAIC_LOG_VERBOSE("Not all channel ready!");
            return;
        }
        sending_ = true;
    }

    std::thread([this, msg] {
        try {
            this->ProcessMsg(msg);
        } catch (const std::runtime_error &e) {
            MOSAIC_LOG_ERROR("[PointCloud2Sender] Error while ProcessMsg: {}", e.what());
        }

        {
            std::lock_guard lock(mutex_);
            sending_ = false;
        }
    }).detach();
}

bool PointCloud2Sender::IsAllChannelReady() const {
    if (data_channels_.empty()) {
        return false;
    }
    for (const auto &channel: data_channels_) {
        if (channel->Sendable()) {
            return true;
        }
    }
    return false;
}

std::shared_ptr<PointCloud2DataChannel> PointCloud2Sender::GetNextChannel(int retry_count) {
    channel_idx_++;
    channel_idx_ %= max_channel_idx_;
    if (data_channels_[channel_idx_]->Sendable()) {
        return data_channels_[channel_idx_];
    }
    if (retry_count > 10) {
        MOSAIC_LOG_ERROR("Failed to get next channel after 10 retries!");
        return nullptr;
    }
    MOSAIC_LOG_INFO("Try find another channel!");
    // 10 ms sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return GetNextChannel(++retry_count);
}

void PointCloud2Sender::SendData(const std::string &data) {
    if (data.length() > 251 * 1024) {
        MOSAIC_LOG_ERROR("Data too long!: {}", data.length());
    }
    if (const auto channel = GetNextChannel(0); channel != nullptr) {
        channel->SendStringAsByte(data, true);
    }
}
