//
// Created by yhkim on 1/1/26.
//

#include <mosaic-ros2-sensor/connector/image_connector.hpp>

#ifdef ROS_DISTRO_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

using namespace mosaic::ros2::sensor_connector;

void ImageConnectorConfigurer::Configure() {
    MOSAIC_LOG_INFO("Configuring ROS2 sensor_msgs::Image Connector...");

    handler_ = std::make_shared<ImageMediaTrack>(connector_config_->label, mosaic_node_);

    const auto subscription = mosaic_node_->create_subscription<sensor_msgs::msg::Image>(
        connector_config_->params.at("topic_name"),
        10,
        std::bind(&ImageConnectorConfigurer::Callback, this, std::placeholders::_1));

    mosaic_node_->AddSubscription(subscription);
    MOSAIC_LOG_INFO("Configuring ROS2 sensor_msgs::Image Connector Done!");
}

void ImageConnectorConfigurer::Callback(sensor_msgs::msg::Image::SharedPtr msg) {
    MOSAIC_LOG_VERBOSE("sensor_msgs::msg::Image received");

    if (handler_) {
        if (const auto image_handler = std::dynamic_pointer_cast<ImageMediaTrack>(handler_)) {
            image_handler->OnImageReceived(msg);
        }
    }
}

void ImageMediaTrack::Start() {
    if (IsRunning()) {
        MOSAIC_LOG_ERROR("ImageMediaTrack is already running");
        return;
    }

    SetRunning(true);
    start_time_ = std::chrono::steady_clock::now();

    MOSAIC_LOG_INFO("ImageMediaTrack started");
}

void ImageMediaTrack::Stop() {
    if (!IsRunning()) {
        return;
    }

    SetStopFlag(true);
    SetRunning(false);
    MOSAIC_LOG_INFO("ImageMediaTrack stopped");
}

void ImageMediaTrack::OnImageReceived(const sensor_msgs::msg::Image::SharedPtr &image) {
    if (!IsRunning()) {
        return;
    }
    ProcessAsync(image);
}

void ImageMediaTrack::ProcessAsync(const sensor_msgs::msg::Image::SharedPtr &msg) {
    std::thread([this, msg] {
        // Convert ROS image to WebRTC VideoFrame
        if (const auto cv_image = RosImageToCvMat(msg); !cv_image.empty()) {
            SendFrame(cv_image, start_time_);
        }
    }).detach();
}

cv::Mat mosaic::ros2::sensor_connector::RosImageToCvMat(const sensor_msgs::msg::Image::SharedPtr &msg) {
    if (!msg) {
        return cv::Mat();
    }

    // Convert ROS Image to OpenCV Mat using cv_bridge
    try {
        const cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        return cv_ptr->image;
    } catch (cv_bridge::Exception &e) {
        MOSAIC_LOG_ERROR("cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}
