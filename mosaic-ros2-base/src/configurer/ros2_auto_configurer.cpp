//
// Created by yhkim on 1/2/26.
//

#include <mosaic-ros2-base/configurer/ros2_auto_configurer.hpp>
#include <mosaic-ros2-base/configurer/ros2_connector_configurer.hpp>

using namespace mosaic::ros2;

void ROS2AutoConfigurer::BeforeConfigure() {
    for (const auto &configurable_connector: configurable_connectors_) {
        if (const auto ros2_configurer = std::dynamic_pointer_cast<ROS2ConnectorConfigurer>(configurable_connector)) {
            ros2_configurer->SetMosaicNode(mosaic_node_);
        }
    }
}
