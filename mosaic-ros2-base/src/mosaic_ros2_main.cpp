#include <mosaic/auto_configurer/config_reader/config_reader_resolver.h>
#include <mosaic/auto_configurer/config_reader/yaml_config_reader.h>
#include <mosaic/auto_configurer/connector/connector_resolver.h>
#include <mosaic/logger/logger.h>

#include "mosaic_ros2/configurer/ros2_auto_configurer.h"
#include "mosaic_ros2/geometry_msgs/twist_connector.h"
#include "mosaic_ros2/geometry_msgs/twist_stamped_connector.h"
#include "mosaic_ros2/logger/ros_logger.h"
#include "mosaic_ros2/node/mosaic_node.h"
#include "mosaic_ros2/sensor_msgs/image_connector.h"
#include "mosaic_ros2/sensor_msgs/nav_sat_fix_connector.h"
#include "rclcpp/rclcpp.hpp"

std::atomic<bool> shutdown_flag(false);

void signal_handler(int signal) {
    std::cerr << "Interrupt signal (" << signal << ") received." << std::endl;
    shutdown_flag.store(true);
}

struct Parameters {
    std::string mosaic_config_path;
    std::string mosaic_log_level;
    std::string webrtc_log_level;
};

std::shared_ptr<Parameters> GetParameters();

void AutoRegisterConnectors();

void SetMOSAICLog(const std::shared_ptr<Parameters>& parameters, rclcpp::Logger logger);

void SetWebRTCLog(const std::shared_ptr<Parameters>& parameters);

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    mosaic::auto_configurer::ConfigReaderResolver::GetInstance()
        .RegisterConfigReader<mosaic::auto_configurer::YamlConfigReader>();

    AutoRegisterConnectors();

    rclcpp::init(argc, argv);

    const auto parameters = GetParameters();

    SetWebRTCLog(parameters);

    const auto node = std::make_shared<mosaic::ros2::MosaicNode>();
    SetMOSAICLog(parameters, node->get_logger());

    const auto auto_configurer = std::make_shared<mosaic::ros2::ROS2AutoConfigurer>();
    auto_configurer->SetMosaicNode(node);
    auto_configurer->AutoConfigure(parameters->mosaic_config_path);

    std::thread ros_thread([node]() {
        while (rclcpp::ok() && !shutdown_flag.load()) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    while (rclcpp::ok() && !shutdown_flag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    MOSAIC_LOG_INFO("Shutting down...");

    try {
        auto_configurer->GetMosaicConnector()->ShuttingDown();
    } catch (...) {
    }

    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}

std::shared_ptr<Parameters> GetParameters() {
    const auto param_node = rclcpp::Node::make_shared("param_node");
    param_node->declare_parameter<std::string>("mosaic_config", "./mosaic_config.yaml");
    param_node->declare_parameter<std::string>("mosaic_log_level", "info");
    param_node->declare_parameter<std::string>("webrtc_log_level", "none");

    const auto parameters = std::make_shared<Parameters>();
    parameters->mosaic_config_path = param_node->get_parameter("mosaic_config").as_string();
    parameters->mosaic_log_level = param_node->get_parameter("mosaic_log_level").as_string();
    parameters->webrtc_log_level = param_node->get_parameter("webrtc_log_level").as_string();

    return parameters;
}

void AutoRegisterConnectors() {
    mosaic::auto_configurer::ConnectorResolver::GetInstance()
        .RegisterConfigurableConnector<mosaic::ros2::geometry_connector::TwistConnectorConfigurer>();
    mosaic::auto_configurer::ConnectorResolver::GetInstance()
        .RegisterConfigurableConnector<mosaic::ros2::geometry_connector::TwistStampedConnectorConfigurer>();

    mosaic::auto_configurer::ConnectorResolver::GetInstance()
        .RegisterConfigurableConnector<mosaic::ros2::sensor_connector::ImageConnectorConfigurer>();
    mosaic::auto_configurer::ConnectorResolver::GetInstance()
        .RegisterConfigurableConnector<mosaic::ros2::sensor_connector::NavSatFixConnectorConfigurer>();
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
