//
// Created by yhkim on 1/19/26.
//

#include <json/json.h>
#include <mosaic_ros2/sensor_msgs/point_cloud2_connector.h>

using namespace mosaic::ros2::sensor_connector;

void PointCloud2ConnectorConfigurer::Configure(const std::shared_ptr<core::MosaicConnector> mosaic_connector) {
    MOSAIC_LOG_INFO("Configuring ROS2 sensor_msgs::PointCloud2 Connector...");

    handler_ = std::make_shared<PointCloud2DataChannel>(connector_config_.label, mosaic_node_);
    mosaic_connector->AddDataChannelHandler(handler_);

    subscription_ = mosaic_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        connector_config_.params.at("topic_name"),
        10,
        std::bind(&PointCloud2ConnectorConfigurer::Callback, this, std::placeholders::_1));

    mosaic_node_->AddSubscription(subscription_);
    MOSAIC_LOG_INFO("Configuring ROS2 sensor_msgs::PointCloud2 Connector Done!");
}

void PointCloud2ConnectorConfigurer::Callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    MOSAIC_LOG_DEBUG("sensor_msgs::msg::PointCloud2 received");

    if (handler_) {
        if (const auto nav_sat_fix_handler = std::dynamic_pointer_cast<PointCloud2DataChannel>(handler_)) {
            nav_sat_fix_handler->OnPointCloud2Received(msg);
        }
    }
}

void PointCloud2DataChannel::OnPointCloud2Received(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud2) {
    if (!Sendable()) {
        MOSAIC_LOG_DEBUG("PointCloud2DataChannel Not Sendable!");
        return;
    }

    const auto now = std::chrono::high_resolution_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

    SendJson("coordinate_json");
}
