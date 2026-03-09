//
// Created by yhkim on 1/19/26.
//

#ifndef MOSAIC_ROS2_SENSOR_POINT_CLOUD_2_CONNECTOR_HPP
#define MOSAIC_ROS2_SENSOR_POINT_CLOUD_2_CONNECTOR_HPP

#include <mosaic-ros2-base/configurer/ros2_a_multiple_dc_handler_configurer.hpp>
#include <mosaic-ros2-base/node/mosaic_node.hpp>
#include <mosaic/handlers/data_channel/data_channel_sendable.hpp>

#include <atomic>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mosaic::ros2::sensor_connector {
    class PointCloud2Sender;
    class PointCloud2DataChannel;

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

        std::atomic<bool> running_{false};
    };

    class PointCloud2Sender {
    public:
        virtual ~PointCloud2Sender() = default;

        void AddPointCloud2DataChannel(const std::shared_ptr<PointCloud2DataChannel> &channel);

        virtual void SetParams(const std::unordered_map<std::string, std::string> &params) = 0;

        virtual void ProcessAsync(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) = 0;

    protected:
        bool IsAllChannelReady() const;

        std::shared_ptr<PointCloud2DataChannel> GetNextChannel(int retry_count);

        void SendData(const std::string &data);

    private:
        int channel_idx_ = 0;
        int max_channel_idx_ = 0;
        std::vector<std::shared_ptr<PointCloud2DataChannel> > point_cloud_2_data_channels_;
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
