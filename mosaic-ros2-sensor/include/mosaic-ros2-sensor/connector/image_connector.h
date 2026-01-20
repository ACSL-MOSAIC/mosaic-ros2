//
// Created by yhkim on 1/1/26.
//

#ifndef MOSAIC_ROS2_IMAGE_H
#define MOSAIC_ROS2_IMAGE_H

#include <mosaic-ros2-base/configurer/ros2_a_mt_handler_configurer.h>
#include <mosaic-ros2-base/node/mosaic_node.h>
#include <mosaic/handlers/media_track/a_media_track_handler.h>

#include "sensor_msgs/msg/image.hpp"

namespace mosaic::ros2::sensor_connector {
cv::Mat RosImageToCvMat(const sensor_msgs::msg::Image::SharedPtr& msg);

class ImageConnectorConfigurer : public ROS2AMTHandlerConfigurer {
  public:
    ImageConnectorConfigurer() = default;

    std::string GetConnectorType() const override {
        return "ros2-sender-sensor-Image";
    }

    void Configure() override;

    void Callback(sensor_msgs::msg::Image::SharedPtr msg);

  private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscription_;
};

class ImageMediaTrack : public handlers::AMediaTrackHandler {
  public:
    explicit ImageMediaTrack(const std::string& track_name, const std::shared_ptr<MosaicNode>& mosaic_node)
        : AMediaTrackHandler(track_name, false), mosaic_node_(mosaic_node) {}

    ~ImageMediaTrack() override {
        ImageMediaTrack::Stop();
    }

    void Start() override;

    void Stop() override;

    void OnImageReceived(const sensor_msgs::msg::Image::SharedPtr& image);

  private:
    void ConvertingLoop();

    std::shared_ptr<MosaicNode> mosaic_node_;

    std::unique_ptr<std::thread> converting_loop_thread_;
    std::chrono::steady_clock::time_point start_time_;
    std::mutex node_mutex_;

    sensor_msgs::msg::Image::SharedPtr last_image_;
    bool changed_ = false;
};
}  // namespace mosaic::ros2::sensor_connector

#endif  // MOSAIC_ROS2_IMAGE_H
