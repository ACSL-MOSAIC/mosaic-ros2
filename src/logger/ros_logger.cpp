//
// Created by yhkim on 1/1/26.
//

#include "mosaic_ros2/logger/ros_logger.h"

using namespace mosaic;

void RosLogger::LOG(const std::string& message, const core_log::LogLevel log_level) {
    switch (log_level) {
        case core_log::DEBUG:
            RCLCPP_DEBUG(logger_, "%s", message.c_str());
            break;
        case core_log::WARNING:
            RCLCPP_WARN(logger_, "%s", message.c_str());
            break;
        case core_log::INFO:
            RCLCPP_INFO(logger_, "%s", message.c_str());
            break;
        case core_log::ERROR:
            RCLCPP_ERROR(logger_, "%s", message.c_str());
            break;
    }
}
