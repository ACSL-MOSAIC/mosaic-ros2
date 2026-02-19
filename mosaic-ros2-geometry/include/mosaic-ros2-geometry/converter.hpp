//
// Created by yhkim on 2/8/26.
//

#ifndef MOSAIC_ROS2_GEOMETRY_CONVERTER_HPP
#define MOSAIC_ROS2_GEOMETRY_CONVERTER_HPP

#include <json/value.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace mosaic::ros2::geometry_connector {
    geometry_msgs::msg::Vector3 JsonToVector3(Json::Value json);

    Json::Value PointToJson(geometry_msgs::msg::Point point);

    Json::Value QuaternionToJson(geometry_msgs::msg::Quaternion quaternion);
} // namespace mosaic::ros2::geometry_connector

#endif  // MOSAIC_ROS2_GEOMETRY_CONVERTER_HPP
