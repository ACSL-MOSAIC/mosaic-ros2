//
// Created by yhkim on 1/1/26.
//

#include "mosaic-ros2/mosaic_node.h"

using namespace mosaic;

MosaicNode::MosaicNode() : Node("mosaic_node") {
    RCLCPP_INFO(this->get_logger(), "MosaicNode has been created.");
}

void MosaicNode::AddPublisher(const std::shared_ptr<rclcpp::PublisherBase>& publisher) {
    publishers_.push_back(publisher);
}

void MosaicNode::AddSubscription(const std::shared_ptr<rclcpp::SubscriptionBase>& subscription) {
    subscriptions_.push_back(subscription);
}
