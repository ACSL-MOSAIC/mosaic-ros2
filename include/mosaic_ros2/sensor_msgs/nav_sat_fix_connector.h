//
// Created by yhkim on 1/5/26.
//

#ifndef MOSAIC_ROS2_NAV_SAT_FIX_CONNECTOR_H
#define MOSAIC_ROS2_NAV_SAT_FIX_CONNECTOR_H

#include <mosaic_auto_configurer/connector/i_dc_handler_configurer.h>
#include <mosaic_rtc_core/handlers/data_channel/data_channel_sendable.h>

#include "mosaic_ros2/ros2_connector_configurer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace mosaic::ros2::sensor_connector {
class NavSatFixConnectorConfigurer : public auto_configurer::IDCHandlerConfigurer, public ROS2ConnectorConfigurer {
  public:
    NavSatFixConnectorConfigurer() = default;

    std::string GetConnectorType() const override {
        return "ros2-sender-sensor-NavSatFix";
    }

    void Configure(std::shared_ptr<core::MosaicConnector> mosaic_connector) override;

    void Callback(sensor_msgs::msg::NavSatFix::SharedPtr msg);

  private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>> subscription_;
};

class NavSatFixDataChannel : public handlers::DataChannelSendable {
  public:
    explicit NavSatFixDataChannel(const std::string& channel_name, const std::shared_ptr<MosaicNode>& mosaic_node)
        : DataChannelSendable(channel_name), mosaic_node_(mosaic_node) {}

    ~NavSatFixDataChannel() override = default;

    void OnNavSatFixReceived(const sensor_msgs::msg::NavSatFix::SharedPtr& nav_sat_fix);

  private:
    std::shared_ptr<MosaicNode> mosaic_node_;
};

}  // namespace mosaic::ros2::sensor_connector

#endif  // MOSAIC_ROS2_NAV_SAT_FIX_CONNECTOR_H
