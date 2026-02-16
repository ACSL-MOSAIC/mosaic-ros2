//
// Created by yhkim on 1/20/26.
//

#ifndef MOSAIC_ROS2_BASE_ROS2_A_DC_HANDLER_CONFIGURER_HPP
#define MOSAIC_ROS2_BASE_ROS2_A_DC_HANDLER_CONFIGURER_HPP

#include "mosaic/auto_configurer/connector/configurable_connectors.hpp"
#include "ros2_connector_configurer.hpp"

namespace mosaic::ros2 {
    class ROS2ADCHandlerConfigurer : public auto_configurer::ADCHandlerConfigurer, public ROS2ConnectorConfigurer {
    public:
        ROS2ADCHandlerConfigurer() = default;

        virtual std::string GetConnectorType() const override = 0;

        virtual void Configure() override = 0;

        virtual void ValidateConfig() override {
            ValidateROS2Config(connector_config_);
        }
    };
} // namespace mosaic::ros2

#endif  // MOSAIC_ROS2_BASE_ROS2_A_DC_HANDLER_CONFIGURER_HPP
