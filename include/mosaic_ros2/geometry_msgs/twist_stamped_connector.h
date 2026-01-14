//
// Created by yhkim on 1/12/26.
//

#ifndef MOSAIC_ROS2_TWIST_STAMPED_CONNECTOR_H
#define MOSAIC_ROS2_TWIST_STAMPED_CONNECTOR_H

#include <mosaic/auto_configurer/connector/i_dc_handler_configurer.h>
#include <mosaic/handlers/data_channel/data_channel_receivable.h>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mosaic_ros2/ros2_connector_configurer.h"
#include "rclcpp/rclcpp.hpp"

using SharedTwistStampedPublisher = std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>>;

namespace mosaic::ros2::geometry_connector {
class TwistStampedConnectorConfigurer : public auto_configurer::IDCHandlerConfigurer, public ROS2ConnectorConfigurer {
  public:
    TwistStampedConnectorConfigurer() = default;

    std::string GetConnectorType() const override {
        return "ros2-receiver-geometry-TwistStamped";
    }

    void Configure(std::shared_ptr<core::MosaicConnector> mosaic_connector) override;

  private:
    SharedTwistStampedPublisher publisher_;
};

struct TwistStampedMessage {
    std::string direction;
};

class TwistStampedDataChannel : public handlers::DataChannelReceivable<TwistStampedMessage> {
  public:
    explicit TwistStampedDataChannel(const std::string& channel_name, const SharedTwistStampedPublisher& publisher)
        : DataChannelReceivable(channel_name), publisher_(publisher) {}

    ~TwistStampedDataChannel() override = default;

    TwistStampedMessage ConvertJsonToData(const Json::Value& json_data) override;

    void HandleData(const TwistStampedMessage& data) override;

    void SendTwist(const double& linear_x, const double& angular_z) const;

  private:
    SharedTwistStampedPublisher publisher_;
};

}  // namespace mosaic::ros2::geometry_connector

#endif  // MOSAIC_ROS2_TWIST_STAMPED_CONNECTOR_H
