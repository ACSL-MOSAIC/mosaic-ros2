//
// Created by yhkim on 1/1/26.
//

#ifndef MOSAIC_ROS2_ROS2_CONFIGURER_H
#define MOSAIC_ROS2_ROS2_CONFIGURER_H

#include <memory>

#include "mosaic_node.h"

namespace mosaic::ros2 {
class ROS2Configurer {
  public:
    ROS2Configurer() = default;

    virtual ~ROS2Configurer() = default;

    void SetMosaicNode(std::shared_ptr<MosaicNode> mosaic_node) {
        mosaic_node_ = std::move(mosaic_node);
    }

    std::shared_ptr<MosaicNode> mosaic_node_;
};
}  // namespace mosaic::ros2

#endif  // MOSAIC_ROS2_ROS2_CONFIGURER_H
