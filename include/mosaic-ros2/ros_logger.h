//
// Created by yhkim on 1/1/26.
//

#ifndef BUILD_ROS_LOGGER_H
#define BUILD_ROS_LOGGER_H

#include <mosaic_rtc_core/logger/logger.h>

#include "rclcpp/rclcpp.hpp"

namespace mosaic {
class RosLogger : public core_log::ILogger {
  public:
    RosLogger(rclcpp::Logger logger) : logger_(std::move(logger)) {}

    void LOG(const std::string& message, core_log::LogLevel log_level) override;

  private:
    rclcpp::Logger logger_;
};
}  // namespace mosaic

#endif  // BUILD_ROS_LOGGER_H