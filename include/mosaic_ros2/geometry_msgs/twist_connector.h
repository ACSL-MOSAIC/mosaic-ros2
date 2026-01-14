//
// Created by yhkim on 1/5/26.
//

#ifndef MOSAIC_ROS2_TWIST_CONNECTOR_H
#define MOSAIC_ROS2_TWIST_CONNECTOR_H

#include <mosaic/auto_configurer/connector/i_dc_handler_configurer.h>
#include <mosaic/handlers/data_channel/data_channel_receivable.h>

#include "geometry_msgs/msg/twist.hpp"
#include "mosaic_ros2/ros2_connector_configurer.h"
#include "rclcpp/rclcpp.hpp"

using SharedTwistPublisher = std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>;

namespace mosaic::ros2::geometry_connector {
class TwistConnectorConfigurer : public auto_configurer::IDCHandlerConfigurer, public ROS2ConnectorConfigurer {
  public:
    TwistConnectorConfigurer() = default;

    std::string GetConnectorType() const override {
        return "ros2-receiver-geometry-Twist";
    }

    void Configure(std::shared_ptr<core::MosaicConnector> mosaic_connector) override;

  private:
    SharedTwistPublisher publisher_;
};

struct TwistMessage {
    std::string direction;
};

class TwistDataChannel : public handlers::DataChannelReceivable<TwistMessage> {
  public:
    explicit TwistDataChannel(const std::string& channel_name, const SharedTwistPublisher& publisher)
        : DataChannelReceivable(channel_name), publisher_(publisher) {}

    ~TwistDataChannel() override = default;

    TwistMessage ConvertJsonToData(const Json::Value& json_data) override;

    void HandleData(const TwistMessage& data) override;

    void SendTwist(const double& linear_x, const double& angular_z) const;

  private:
    SharedTwistPublisher publisher_;
};

}  // namespace mosaic::ros2::geometry_connector

#endif  // MOSAIC_ROS2_TWIST_CONNECTOR_H
