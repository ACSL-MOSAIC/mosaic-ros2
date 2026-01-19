//
// Created by yhkim on 1/19/26.
//

#include "mosaic_ros2/sensor_msgs/point_cloud2_connector.h"

#include <json/json.h>

#include "mosaic_ros2/sensor_msgs/util/point_cloud2_sender.h"

using namespace mosaic::ros2::sensor_connector;

void PointCloud2ConnectorConfigurer::Configure() {
    MOSAIC_LOG_INFO("Configuring ROS2 sensor_msgs::PointCloud2 Connector...");

    // TODO: voxel size, MIN_POINTS_TO_SUBDIVIDE to configurable params
    point_cloud2_sender_ = std::make_unique<PointCloud2Sender>(0.5);

    for (int i = 0; i < parallel_num_; i++) {
        const auto channel_label = connector_config_.label + "_" + std::to_string(i);
        const auto handler = std::make_shared<PointCloud2DataChannel>(channel_label, mosaic_node_);
        handlers_.push_back(handler);
        point_cloud2_sender_->AddPointCloud2DataChannel(handler);
    }

    subscription_ = mosaic_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name_, 10, std::bind(&PointCloud2ConnectorConfigurer::Callback, this, std::placeholders::_1));

    mosaic_node_->AddSubscription(subscription_);
    MOSAIC_LOG_INFO("Configuring ROS2 sensor_msgs::PointCloud2 Connector Done!");
}

void PointCloud2ConnectorConfigurer::Callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    MOSAIC_LOG_DEBUG("sensor_msgs::msg::PointCloud2 received");

    try {
        point_cloud2_sender_->Send(msg);
    } catch (const std::exception& e) {
        MOSAIC_LOG_ERROR("Failed to send PointCloud2: {}", e.what());
    }
}
