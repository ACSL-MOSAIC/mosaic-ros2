//
// Created by yhkim on 1/5/26.
//

#ifndef MOSAIC_ROS2_TWIST_CONNECTOR_H
#define MOSAIC_ROS2_TWIST_CONNECTOR_H

#include <mosaic/handlers/data_channel/data_channel_receivable.h>

#include "geometry_msgs/msg/twist.hpp"
#include "mosaic_ros2/configurer/ros2_a_dc_handler_configurer.h"

using SharedTwistPublisher = std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>;

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
