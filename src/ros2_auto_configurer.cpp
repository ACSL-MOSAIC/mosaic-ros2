//
// Created by yhkim on 1/2/26.
//

#include <mosaic_ros2/ros2_auto_configurer.h>
#include <mosaic_ros2/ros2_connector_configurer.h>

using namespace mosaic::ros2;

void ROS2AutoConfigurer::BeforeConfigure() {
    ValidateConnectorConfigs();

    for (const auto& configurable_connector : configurable_connectors_) {
        if (const auto ros2_configurer = std::dynamic_pointer_cast<ROS2ConnectorConfigurer>(configurable_connector)) {
            ros2_configurer->SetMosaicNode(mosaic_node_);
        }
    }
}

void ROS2AutoConfigurer::ValidateConnectorConfigs() {
    // TODO
    // check if connector configs have "topic_name" for subscribe/publish
}