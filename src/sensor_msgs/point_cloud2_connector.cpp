//
// Created by yhkim on 1/19/26.
//

#include <json/json.h>
#include <mosaic_ros2/sensor_msgs/point_cloud2_connector.h>

using namespace mosaic::ros2::sensor_connector;

void PointCloud2ConnectorConfigurer::Configure() {
    MOSAIC_LOG_INFO("Configuring ROS2 sensor_msgs::PointCloud2 Connector...");

    point_cloud2_sender_ = std::make_unique<PointCloud2Sender>(0.5);

    int parallel_num = 1;
    if (connector_config_.params.find("parallel_num") != connector_config_.params.end()) {
        parallel_num = std::stoi(connector_config_.params.at("parallel_num"));
    }

    for (int i = 0; i < parallel_num; i++) {
        const auto channel_label = connector_config_.label + "_" + std::to_string(i);
        const auto handler = std::make_shared<PointCloud2DataChannel>(channel_label, mosaic_node_);
        handlers_.push_back(handler);
        point_cloud2_sender_->AddPointCloud2DataChannel(handler);
    }

    subscription_ = mosaic_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        connector_config_.params.at("topic_name"),
        10,
        std::bind(&PointCloud2ConnectorConfigurer::Callback, this, std::placeholders::_1));

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
