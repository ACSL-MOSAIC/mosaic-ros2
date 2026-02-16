//
// Created by yhkim on 1/5/26.
//

#include <mosaic-ros2-geometry/connector/twist_connector.hpp>
#include <mosaic-ros2-geometry/converter.hpp>

using namespace mosaic::ros2::geometry_connector;

void TwistConnectorConfigurer::Configure() {
    MOSAIC_LOG_INFO("Configuring ROS2 geometry_msgs::Twist Connector...");

    publisher_ =
            mosaic_node_->create_publisher<geometry_msgs::msg::Twist>(connector_config_->params.at("topic_name"), 10);

    handler_ = std::make_shared<TwistDataChannel>(connector_config_->label, publisher_);

    mosaic_node_->AddPublisher(publisher_);
}

void TwistDataChannel::HandleData(const Json::Value &data) {
    if (!publisher_) {
        return;
    }

    geometry_msgs::msg::Twist twist_msg;
    if (data["linear"]) {
        twist_msg.linear = JsonToVector3(data["linear"]);
    }
    if (data["angular"]) {
        twist_msg.angular = JsonToVector3(data["angular"]);
    }
    publisher_->publish(twist_msg);
}
