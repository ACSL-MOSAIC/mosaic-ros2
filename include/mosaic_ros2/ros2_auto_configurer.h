//
// Created by yhkim on 1/2/26.
//

#ifndef BUILD_ROS2_CONFIGURER_H
#define BUILD_ROS2_CONFIGURER_H

#include <mosaic_auto_configurer/auto_configurer.h>

#include "mosaic_node.h"

namespace mosaic::ros2 {

class ROS2AutoConfigurer : public auto_configurer::AutoConfigurer {
  public:
    ROS2AutoConfigurer() = default;

    void BeforeConfigure() override;

    void SetMosaicNode(std::shared_ptr<MosaicNode> mosaic_node) {
        mosaic_node_ = std::move(mosaic_node);
    }

  private:
    std::shared_ptr<MosaicNode> mosaic_node_;
};

}  // namespace mosaic::ros2

#endif  // BUILD_ROS2_CONFIGURER_H
