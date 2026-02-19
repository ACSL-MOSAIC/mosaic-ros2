//
// Created by yhkim on 1/1/26.
//

#ifndef MOSAIC_ROS2_BASE_ROS_LOGGER_HPP
#define MOSAIC_ROS2_BASE_ROS_LOGGER_HPP

#include <mosaic/logger/logger.hpp>

#include "rclcpp/rclcpp.hpp"

namespace mosaic {
  class RosLogger : public core_log::ILogger {
  public:
    explicit RosLogger(rclcpp::Logger logger) : logger_(std::move(logger)) {
    }

    void LOG(const std::string &message, core_log::LogLevel log_level) override;

  private:
    rclcpp::Logger logger_;
  };
} // namespace mosaic

#endif  // MOSAIC_ROS2_BASE_ROS_LOGGER_HPP
