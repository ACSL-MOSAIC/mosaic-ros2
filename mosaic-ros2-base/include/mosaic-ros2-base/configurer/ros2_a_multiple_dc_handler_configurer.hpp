//
// Created by yhkim on 1/20/26.
//

#ifndef MOSAIC_ROS2_BASE_ROS2_A_PARALLEL_DC_HANDLER_CONFIGURER_HPP
#define MOSAIC_ROS2_BASE_ROS2_A_PARALLEL_DC_HANDLER_CONFIGURER_HPP

#include "mosaic/auto_configurer/connector/configurable_connectors.hpp"
#include "ros2_connector_configurer.hpp"

namespace mosaic::ros2 {
    class ROS2AMultipleDCHandlerConfigurer : public auto_configurer::AMultipleDCHandlerConfigurer,
                                             public ROS2ConnectorConfigurer {
    public:
        ROS2AMultipleDCHandlerConfigurer() = default;

        virtual std::string GetConnectorType() const override = 0;

        virtual void ValidateConfig() override {
            ValidateROS2Config(connector_config_);
            if (connector_config_->params.find("parallel_num") != connector_config_->params.end()) {
                parallel_num_ = std::stoi(connector_config_->params.at("parallel_num"));
            }
        }

        virtual void Configure() override = 0;

    protected:
        int parallel_num_ = 1;
    };
} // namespace mosaic::ros2

#endif  // MOSAIC_ROS2_BASE_ROS2_A_PARALLEL_DC_HANDLER_CONFIGURER_HPP
