//
// Created by yhkim on 1/19/26.
//

#include "mosaic-ros2-base/configurer/ros2_connector_configurer.hpp"

using namespace mosaic::ros2;

void ROS2ConnectorConfigurer::ValidateROS2Config(const std::shared_ptr<core::ConnectorConfig> &connector_config) {
    // check if connector_config.params has "topic_name" for subscribe/publish
    const auto it = connector_config->params.find("topic_name");
    if (it == connector_config->params.end()) {
        throw std::invalid_argument("ROS2 connector requires 'topic_name' parameter");
    }

    if (it->second.empty()) {
        throw std::invalid_argument("ROS2 connector 'topic_name' cannot be empty");
    }

    topic_name_ = it->second;
}
