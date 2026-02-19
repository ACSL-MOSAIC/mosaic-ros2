//
// Created by yhkim on 2/8/26.
//

#ifndef MOSAIC_ROS2_GEOMETRY_POSE_CONNECTOR_H
#define MOSAIC_ROS2_GEOMETRY_POSE_CONNECTOR_H

#include <mosaic-ros2-base/configurer/ros2_a_dc_handler_configurer.hpp>
#include <mosaic/handlers/data_channel/data_channel_sendable.hpp>

#include "geometry_msgs/msg/pose.hpp"

namespace mosaic::ros2::geometry_connector {
    class PoseConnectorConfigurer : public ROS2ADCHandlerConfigurer {
    public:
        PoseConnectorConfigurer() = default;

        std::string GetConnectorType() const override {
            return "ros2-sender-geometry-Pose";
        }

        void Configure() override;

        void Callback(geometry_msgs::msg::Pose::SharedPtr msg);

    private:
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose> > subscription_;
    };

    class PoseDataChannel : public handlers::DataChannelSendable {
    public:
        explicit PoseDataChannel(const std::string &channel_name, const std::shared_ptr<MosaicNode> &mosaic_node)
            : DataChannelSendable(channel_name), mosaic_node_(mosaic_node) {
        }

        ~PoseDataChannel() override = default;

        void OnPoseReceived(const geometry_msgs::msg::Pose::SharedPtr &pose);

    private:
        std::shared_ptr<MosaicNode> mosaic_node_;
    };
} // namespace mosaic::ros2::geometry_connector

#endif  // MOSAIC_ROS2_GEOMETRY_POSE_CONNECTOR_H
