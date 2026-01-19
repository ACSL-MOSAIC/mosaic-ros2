//
// Created by yhkim on 1/19/26.
//

#ifndef MOSAIC_ROS2_POINT_CLOUD_2_CONNECTOR_H
#define MOSAIC_ROS2_POINT_CLOUD_2_CONNECTOR_H

#include <mosaic/auto_configurer/connector/a_parallel_dc_handler_configurer.h>
#include <mosaic/handlers/data_channel/data_channel_sendable.h>

#include "mosaic_ros2/ros2_connector_configurer.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "util/point_cloud_sender.h"

namespace mosaic::ros2::sensor_connector {
class PointCloud2ConnectorConfigurer : public auto_configurer::AParallelDCHandlerConfigurer,
                                       public ROS2ConnectorConfigurer {
  public:
    PointCloud2ConnectorConfigurer() = default;

    std::string GetConnectorType() const override {
        return "ros2-sender-sensor-PointCloud2";
    }

    void Configure() override;

    void Callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);

  private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> subscription_;
    std::shared_ptr<PointCloud2Sender> point_cloud2_sender_;
};

class PointCloud2DataChannel : public handlers::DataChannelSendable {
  public:
    explicit PointCloud2DataChannel(const std::string& channel_name, const std::shared_ptr<MosaicNode>& mosaic_node)
        : DataChannelSendable(channel_name), mosaic_node_(mosaic_node) {}

    ~PointCloud2DataChannel() override = default;

  private:
    std::shared_ptr<MosaicNode> mosaic_node_;
};

}  // namespace mosaic::ros2::sensor_connector

#endif  // MOSAIC_ROS2_POINT_CLOUD_2_CONNECTOR_H
