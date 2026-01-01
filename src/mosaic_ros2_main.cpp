#include <mosaic-ros2/mosaic_node.h>
#include <mosaic-ros2/ros_logger.h>
#include <mosaic_rtc_core/logger/logger.h>

#include "rclcpp/rclcpp.hpp"

std::atomic<bool> shutdown_flag(false);

void signal_handler(int signal) {
    std::cerr << "Interrupt signal (" << signal << ") received." << std::endl;
    shutdown_flag.store(true);
}

struct Parameters {
    std::string mosaic_log_level;
    std::string webrtc_log_level;
};

std::shared_ptr<Parameters> GetParameters();

void SetMOSAICLog(const std::shared_ptr<Parameters>& parameters, rclcpp::Logger logger);
void SetWebRTCLog(const std::shared_ptr<Parameters>& parameters);

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);

    const auto parameters = GetParameters();

    SetWebRTCLog(parameters);

    const auto node = std::make_shared<mosaic::MosaicNode>();
    SetMOSAICLog(parameters, node->get_logger());

    return 0;
}

std::shared_ptr<Parameters> GetParameters() {
    const auto param_node = rclcpp::Node::make_shared("param_node");
    param_node->declare_parameter<std::string>("mosaic_log_level", "info");
    param_node->declare_parameter<std::string>("webrtc_log_level", "none");

    const auto parameters = std::make_shared<Parameters>();
    parameters->mosaic_log_level = param_node->get_parameter("mosaic_log_level").as_string();
    parameters->webrtc_log_level = param_node->get_parameter("webrtc_log_level").as_string();

    return parameters;
}

void SetMOSAICLog(const std::shared_ptr<Parameters>& parameters, rclcpp::Logger logger) {
    mosaic::core_log::RegisterLogger<mosaic::RosLogger>(logger);

    const auto level = parameters->mosaic_log_level;
    if (level.empty()) {
        throw std::invalid_argument("MOSAIC log level is not set");
    }
    if (level == "debug") {
        mosaic::core_log::SetLogLevel(mosaic::core_log::LogLevel::DEBUG);
        const auto ret = rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(logger, "Error setting severity: %s", rcutils_get_error_string().str);
            rcutils_reset_error();
        }
    } else if (level == "warning") {
        mosaic::core_log::SetLogLevel(mosaic::core_log::LogLevel::WARNING);
        const auto ret = rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_WARN);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(logger, "Error setting severity: %s", rcutils_get_error_string().str);
            rcutils_reset_error();
        }
    } else if (level == "info") {
        mosaic::core_log::SetLogLevel(mosaic::core_log::LogLevel::INFO);
        const auto ret = rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_INFO);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(logger, "Error setting severity: %s", rcutils_get_error_string().str);
            rcutils_reset_error();
        }
    } else if (level == "error") {
        mosaic::core_log::SetLogLevel(mosaic::core_log::LogLevel::ERROR);
        const auto ret = rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_ERROR);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(logger, "Error setting severity: %s", rcutils_get_error_string().str);
            rcutils_reset_error();
        }
    } else {
        throw std::invalid_argument("Invalid log level");
    }
}

void SetWebRTCLog(const std::shared_ptr<Parameters>& parameters) {
    mosaic::core_log::SetWebRTCLogThreads(false);
    mosaic::core_log::SetWebRTCLogTimestamps(true);

    const auto level = parameters->webrtc_log_level;
    if (level.empty()) {
        throw std::invalid_argument("WebRTC log level is not set");
    }
    if (level == "info") {
        mosaic::core_log::SetWebRTCLogLevel(mosaic::core_log::WebRTCLogLevel::LS_INFO);
    } else if (level == "warning") {
        mosaic::core_log::SetWebRTCLogLevel(mosaic::core_log::WebRTCLogLevel::LS_WARNING);
    } else if (level == "error") {
        mosaic::core_log::SetWebRTCLogLevel(mosaic::core_log::WebRTCLogLevel::LS_ERROR);
    } else if (level == "none") {
        mosaic::core_log::SetWebRTCLogLevel(mosaic::core_log::WebRTCLogLevel::LS_NONE);
    } else if (level == "verbose") {
        mosaic::core_log::SetWebRTCLogLevel(mosaic::core_log::WebRTCLogLevel::LS_VERBOSE);
    } else {
        throw std::invalid_argument("Invalid log level");
    }
}
