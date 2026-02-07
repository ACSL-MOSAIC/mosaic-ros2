//
// Created by yhkim on 1/5/26.
//

#ifndef MOSAIC_ROS2_TWIST_CONNECTOR_H
#define MOSAIC_ROS2_TWIST_CONNECTOR_H

#include <mosaic-ros2-base/configurer/ros2_a_dc_handler_configurer.h>
#include <mosaic/handlers/data_channel/data_channel_receivable.h>

#include "geometry_msgs/msg/twist.hpp"

using SharedTwistPublisher = std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist> >;

namespace mosaic::ros2::geometry_connector {
    class TwistConnectorConfigurer : public ROS2ADCHandlerConfigurer {
    public:
        TwistConnectorConfigurer() = default;

        std::string GetConnectorType() const override {
            return "ros2-receiver-geometry-Twist";
        }

        void Configure() override;

    private:
        SharedTwistPublisher publisher_;
    };

    class TwistDataChannel : public handlers::DataChannelJsonReceivable {
    public:
        explicit TwistDataChannel(const std::string &channel_name, const SharedTwistPublisher &publisher)
            : DataChannelJsonReceivable(channel_name), publisher_(publisher) {
        }

        ~TwistDataChannel() override = default;

        void HandleData(const Json::Value &data) override;

    private:
        SharedTwistPublisher publisher_;
    };
} // namespace mosaic::ros2::geometry_connector

#endif  // MOSAIC_ROS2_TWIST_CONNECTOR_H
