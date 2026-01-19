//
// Created by yhkim on 1/5/26.
//

#include <json/json.h>
#include <mosaic_ros2/geometry_msgs/twist_stamped_connector.h>

using namespace mosaic::ros2::geometry_connector;

void TwistStampedConnectorConfigurer::Configure() {
    MOSAIC_LOG_INFO("Configuring ROS2 geometry_msgs::TwistStamped Connector...");

    publisher_ =
        mosaic_node_->create_publisher<geometry_msgs::msg::TwistStamped>(connector_config_.params.at("topic_name"), 10);

    handler_ = std::make_shared<TwistStampedDataChannel>(connector_config_.label, publisher_);

    mosaic_node_->AddPublisher(publisher_);
}

TwistStampedMessage TwistStampedDataChannel::ConvertJsonToData(const Json::Value& json_data) {
    TwistStampedMessage message;
    if (json_data.isMember("direction") && json_data["direction"].isString()) {
        message.direction = json_data["direction"].asString();
    } else {
        MOSAIC_LOG_ERROR("Invalid remote control message: 'direction' field is missing or not a string.");
        return message;
    }
    return message;
}

void TwistStampedDataChannel::HandleData(const TwistStampedMessage& data) {
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

void TwistStampedDataChannel::SendTwist(const double& linear_x, const double& angular_z) const {
    if (!publisher_) {
        return;
    }

    geometry_msgs::msg::TwistStamped twist_stamped_msg;

    twist_stamped_msg.twist.linear.x = linear_x;
    twist_stamped_msg.twist.angular.z = angular_z;

    publisher_->publish(twist_stamped_msg);
}
