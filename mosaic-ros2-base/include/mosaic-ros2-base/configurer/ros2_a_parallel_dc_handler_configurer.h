//
// Created by yhkim on 1/20/26.
//

#ifndef BUILD_ROS2_A_PARALLEL_DC_HANDLER_CONFIGURER_H
#define BUILD_ROS2_A_PARALLEL_DC_HANDLER_CONFIGURER_H

#include "mosaic/auto_configurer/connector/a_parallel_dc_handler_configurer.h"
#include "ros2_connector_configurer.h"

namespace mosaic::ros2 {
    class ROS2AParallelDCHandlerConfigurer : public auto_configurer::AParallelDCHandlerConfigurer,
                                             public ROS2ConnectorConfigurer {
    public:
        ROS2AParallelDCHandlerConfigurer() = default;

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

#endif  // BUILD_ROS2_A_PARALLEL_DC_HANDLER_CONFIGURER_H
