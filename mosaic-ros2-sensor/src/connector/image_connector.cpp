//
// Created by yhkim on 1/1/26.
//

#include <mosaic-ros2-sensor/connector/image_connector.h>

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
    MOSAIC_LOG_DEBUG("sensor_msgs::msg::Image received");

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
    converting_loop_thread_ = std::make_unique<std::thread>(&ImageMediaTrack::ConvertingLoop, this);

    MOSAIC_LOG_INFO("ImageMediaTrack started");
}

void ImageMediaTrack::Stop() {
    if (!IsRunning()) {
        return;
    }

    SetStopFlag(true);
    if (converting_loop_thread_ && converting_loop_thread_->joinable()) {
        converting_loop_thread_->join();
    }
    converting_loop_thread_.reset();

    SetRunning(false);
    MOSAIC_LOG_INFO("ImageMediaTrack stopped");
}

void ImageMediaTrack::OnImageReceived(const sensor_msgs::msg::Image::SharedPtr &image) {
    std::lock_guard lock(node_mutex_);
    last_image_ = image;
    changed_ = true;
}

void ImageMediaTrack::ConvertingLoop() {
    start_time_ = std::chrono::steady_clock::now();

    while (!GetStopFlag()) {
        if (last_image_ && changed_) {
            std::lock_guard lock(node_mutex_);
            // Convert ROS image to WebRTC VideoFrame
            auto cv_image = RosImageToCvMat(last_image_);
            if (cv_image.empty()) {
                MOSAIC_LOG_ERROR("Failed to convert ROS Image to OpenCV Mat");
                changed_ = false;
                continue;
            }
            SendFrame(cv_image, start_time_);
            changed_ = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Adjust sleep duration as needed
    }
}

cv::Mat mosaic::ros2::sensor_connector::RosImageToCvMat(const sensor_msgs::msg::Image::SharedPtr &msg) {
    if (!msg) {
        return cv::Mat();
    }

    // Convert ROS Image to OpenCV Mat using cv_bridge
    try {
        const cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        return cv_ptr->image;
    } catch (cv_bridge::Exception &e) {
        MOSAIC_LOG_ERROR("cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}
