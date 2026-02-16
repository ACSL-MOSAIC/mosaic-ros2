//
// Created by yhkim on 1/1/26.
//

#ifndef MOSAIC_ROS2_BASE_MOSAIC_NODE_HPP
#define MOSAIC_ROS2_BASE_MOSAIC_NODE_HPP

#include "rclcpp/node.hpp"

namespace mosaic::ros2 {
  class MosaicNode : public rclcpp::Node {
  public:
    explicit MosaicNode();

    void AddPublisher(const std::shared_ptr<rclcpp::PublisherBase> &publisher);

    void AddSubscription(const std::shared_ptr<rclcpp::SubscriptionBase> &subscription);

  private:
    std::vector<std::shared_ptr<rclcpp::PublisherBase> > publishers_;
    std::vector<std::shared_ptr<rclcpp::SubscriptionBase> > subscriptions_;
  };
} // namespace mosaic::ros2

#endif  // MOSAIC_ROS2_BASE_MOSAIC_NODE_HPP
