//
// Created by yhkim on 2/8/26.
//

#include <mosaic-ros2-geometry/connector/pose_connector.hpp>
#include <mosaic-ros2-geometry/converter.hpp>

using namespace mosaic::ros2::geometry_connector;

void PoseConnectorConfigurer::Configure() {
    MOSAIC_LOG_INFO("Configuring ROS2 geometry_msgs::Pose Connector...");

    handler_ = std::make_shared<PoseDataChannel>(connector_config_->label, mosaic_node_);

    subscription_ = mosaic_node_->create_subscription<geometry_msgs::msg::Pose>(
        connector_config_->params.at("topic_name"),
        10,
        std::bind(&PoseConnectorConfigurer::Callback, this, std::placeholders::_1));

    mosaic_node_->AddSubscription(subscription_);
}

void PoseConnectorConfigurer::Callback(geometry_msgs::msg::Pose::SharedPtr msg) {
    MOSAIC_LOG_DEBUG("geometry_msgs::msg::Pose received");

    if (handler_) {
        if (const auto pose_handler = std::dynamic_pointer_cast<PoseDataChannel>(handler_)) {
            pose_handler->OnPoseReceived(msg);
        }
    }
}

void PoseDataChannel::OnPoseReceived(const geometry_msgs::msg::Pose::SharedPtr &pose) {
    if (!Sendable()) {
        MOSAIC_LOG_DEBUG("PoseDataChannel Not Sendable!");
        return;
    }

    const auto now = std::chrono::high_resolution_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

    Json::Value pose_json;
    pose_json["position"] = PointToJson(pose->position);
    pose_json["orientation"] = QuaternionToJson(pose->orientation);
    pose_json["timestamp"] = timestamp;

    SendJson(pose_json);
}
