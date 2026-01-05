//
// Created by yhkim on 1/5/26.
//

#include <json/json.h>
#include <mosaic_ros2/geometry_msgs/twist_connector.h>

using namespace mosaic::ros2::geometry_connector;

void TwistConnectorConfigurer::Configure(const std::shared_ptr<core::MosaicConnector> mosaic_connector) {
    MOSAIC_LOG_INFO("Configuring ROS2 geometry_msgs::Twist Connector...");

    publisher_ =
        mosaic_node_->create_publisher<geometry_msgs::msg::Twist>(connector_config_.params.at("ros2_topic"), 10);

    handler_ = std::make_shared<TwistDataChannel>(connector_config_.label, publisher_);
    mosaic_connector->AddDataChannelHandler(handler_);

    mosaic_node_->AddPublisher(publisher_);
}

TwistMessage TwistDataChannel::ConvertJsonToData(const Json::Value& json_data) {
    TwistMessage message;
    if (json_data.isMember("direction") && json_data["direction"].isString()) {
        message.direction = json_data["direction"].asString();

        // const auto now = std::chrono::high_resolution_clock::now();
        // MOSAIC_LOG_INFO("REMOTE CONTROL MESSAGE RECEIVED: SEND TIMESTAMP: {}, RECEIVE TIMESTAMP: {}",
        // json_data["timestamp"].asLargestUInt(),
        // std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
    } else {
        MOSAIC_LOG_ERROR("Invalid remote control message: 'direction' field is missing or not a string.");
        return message;
    }
    return message;
}

void TwistDataChannel::HandleData(const TwistMessage& data) {
    if (data.direction == "up") {
        SendTwist(2.0, 0.0);
    } else if (data.direction == "down") {
        SendTwist(-2.0, 0.0);
    } else if (data.direction == "left") {
        SendTwist(2.0, 2.0);
    } else if (data.direction == "right") {
        SendTwist(2.0, -2.0);
    } else if (data.direction == "stop") {
        SendTwist(0.0, 0.0);
    } else {
        MOSAIC_LOG_ERROR("Unknown direction command: {}", data.direction);
    }
}

void TwistDataChannel::SendTwist(const double& linear_x, const double& angular_z) const {
    if (!publisher_) {
        return;
    }

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = linear_x;
    twist_msg.angular.z = angular_z;

    publisher_->publish(twist_msg);
}