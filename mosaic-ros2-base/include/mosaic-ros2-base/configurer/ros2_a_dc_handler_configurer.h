//
// Created by yhkim on 1/20/26.
//

#ifndef BUILD_ROS2_A_DC_HANDLER_CONFIGURER_H
#define BUILD_ROS2_A_DC_HANDLER_CONFIGURER_H

#include "mosaic/auto_configurer/connector/a_dc_handler_configurer.h"
#include "ros2_connector_configurer.h"

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
}  // namespace mosaic::ros2

#endif  // BUILD_ROS2_A_DC_HANDLER_CONFIGURER_H
