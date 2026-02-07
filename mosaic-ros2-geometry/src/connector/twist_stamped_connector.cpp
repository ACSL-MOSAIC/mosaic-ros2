//
// Created by yhkim on 1/5/26.
//

#include <json/json.h>
#include <mosaic-ros2-geometry/connector/twist_stamped_connector.h>

#include <mosaic-ros2-geometry/converter.h>

using namespace mosaic::ros2::geometry_connector;

void TwistStampedConnectorConfigurer::Configure() {
    MOSAIC_LOG_INFO("Configuring ROS2 geometry_msgs::TwistStamped Connector...");

    publisher_ =
            mosaic_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
                connector_config_->params.at("topic_name"), 10);

    handler_ = std::make_shared<TwistStampedDataChannel>(connector_config_->label, publisher_);

    mosaic_node_->AddPublisher(publisher_);
}

void TwistStampedDataChannel::HandleData(const Json::Value &data) {
    if (!publisher_) {
        return;
    }

    geometry_msgs::msg::TwistStamped twist_stamped_msg;
    if (data["linear"]) {
        twist_stamped_msg.twist.linear = JsonToVector3(data["linear"]);
    }
    if (data["angular"]) {
        twist_stamped_msg.twist.angular = JsonToVector3(data["angular"]);
    }
    publisher_->publish(twist_stamped_msg);
}
