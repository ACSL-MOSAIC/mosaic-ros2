//
// Created by yhkim on 1/1/26.
//

#ifndef MOSAIC_ROS2_ROS2_CONNECTOR_CONFIGURER_H
#define MOSAIC_ROS2_ROS2_CONNECTOR_CONFIGURER_H

#include <memory>

#include <mosaic/configs_decl.h>

#include "mosaic_ros2/node/mosaic_node.h"

namespace mosaic::ros2 {
class ROS2ConnectorConfigurer {
  public:
    ROS2ConnectorConfigurer() = default;

    void SetMosaicNode(std::shared_ptr<MosaicNode> mosaic_node) {
        mosaic_node_ = std::move(mosaic_node);
    }

    void ValidateROS2Config(const core::ConnectorConfig& connector_config);

  protected:
    std::shared_ptr<MosaicNode> mosaic_node_;
    std::string topic_name_;
};
}  // namespace mosaic::ros2

#endif  // MOSAIC_ROS2_ROS2_CONNECTOR_CONFIGURER_H
