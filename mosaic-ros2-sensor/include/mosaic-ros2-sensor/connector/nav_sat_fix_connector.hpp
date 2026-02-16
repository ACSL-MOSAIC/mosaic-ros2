//
// Created by yhkim on 1/5/26.
//

#ifndef MOSAIC_ROS2_SENSOR_NAV_SAT_FIX_CONNECTOR_HPP
#define MOSAIC_ROS2_SENSOR_NAV_SAT_FIX_CONNECTOR_HPP

#include <mosaic-ros2-base/configurer/ros2_a_dc_handler_configurer.hpp>
#include <mosaic/handlers/data_channel/data_channel_sendable.hpp>

#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace mosaic::ros2::sensor_connector {
    class NavSatFixConnectorConfigurer : public ROS2ADCHandlerConfigurer {
    public:
        NavSatFixConnectorConfigurer() = default;

        std::string GetConnectorType() const override {
            return "ros2-sender-sensor-NavSatFix";
        }

        void Configure() override;

        void Callback(sensor_msgs::msg::NavSatFix::SharedPtr msg);

    private:
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::NavSatFix> > subscription_;
    };

    class NavSatFixDataChannel : public handlers::DataChannelSendable {
    public:
        explicit NavSatFixDataChannel(const std::string &channel_name, const std::shared_ptr<MosaicNode> &mosaic_node)
            : DataChannelSendable(channel_name), mosaic_node_(mosaic_node) {
        }

        ~NavSatFixDataChannel() override = default;

        void OnNavSatFixReceived(const sensor_msgs::msg::NavSatFix::SharedPtr &nav_sat_fix);

    private:
        std::shared_ptr<MosaicNode> mosaic_node_;
    };
} // namespace mosaic::ros2::sensor_connector

#endif  // MOSAIC_ROS2_SENSOR_NAV_SAT_FIX_CONNECTOR_HPP
