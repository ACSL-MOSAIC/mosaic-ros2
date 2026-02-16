//
// Created by yhkim on 1/1/26.
//

#include "mosaic-ros2-base/node/mosaic_node.hpp"

#include "rclcpp/node.hpp"

using namespace mosaic::ros2;

MosaicNode::MosaicNode() : rclcpp::Node("mosaic_node") {
    RCLCPP_INFO(this->get_logger(), "MosaicNode has been created.");
}

void MosaicNode::AddPublisher(const std::shared_ptr<rclcpp::PublisherBase> &publisher) {
    publishers_.push_back(publisher);
}

void MosaicNode::AddSubscription(const std::shared_ptr<rclcpp::SubscriptionBase> &subscription) {
    subscriptions_.push_back(subscription);
}
