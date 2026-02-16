//
// Created by yhkim on 1/2/26.
//

#ifndef MOSAIC_ROS2_BASE_ROS2_CONFIGURER_HPP
#define MOSAIC_ROS2_BASE_ROS2_CONFIGURER_HPP

#include <mosaic/auto_configurer/auto_configurer.hpp>

#include "mosaic-ros2-base/node/mosaic_node.hpp"

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
} // namespace mosaic::ros2

#endif  // MOSAIC_ROS2_BASE_ROS2_CONFIGURER_HPP
