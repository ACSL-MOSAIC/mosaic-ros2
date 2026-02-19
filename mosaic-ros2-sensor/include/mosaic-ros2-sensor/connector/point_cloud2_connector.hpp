//
// Created by yhkim on 1/19/26.
//

#ifndef MOSAIC_ROS2_SENSOR_POINT_CLOUD_2_CONNECTOR_HPP
#define MOSAIC_ROS2_SENSOR_POINT_CLOUD_2_CONNECTOR_HPP

#include <mosaic-ros2-base/configurer/ros2_a_multiple_dc_handler_configurer.hpp>
#include <mosaic-ros2-base/node/mosaic_node.hpp>
#include <mosaic/handlers/data_channel/data_channel_sendable.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mosaic::ros2::sensor_connector {
    class PointCloud2Sender;

    class PointCloud2ConnectorConfigurer : public ROS2AMultipleDCHandlerConfigurer {
    public:
        PointCloud2ConnectorConfigurer() = default;

        std::string GetConnectorType() const override {
            return "ros2-sender-sensor-PointCloud2";
        }

        void Configure() override;

        void Callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);

    private:
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2> > subscription_;
        std::shared_ptr<PointCloud2Sender> point_cloud2_sender_;
    };

    class PointCloud2DataChannel : public handlers::DataChannelSendable {
    public:
        explicit PointCloud2DataChannel(const std::string &channel_name, const std::shared_ptr<MosaicNode> &mosaic_node)
            : DataChannelSendable(channel_name), mosaic_node_(mosaic_node) {
        }

        ~PointCloud2DataChannel() override = default;

    private:
        std::shared_ptr<MosaicNode> mosaic_node_;
    };
} // namespace mosaic::ros2::sensor_connector

#endif  // MOSAIC_ROS2_SENSOR_POINT_CLOUD_2_CONNECTOR_HPP
