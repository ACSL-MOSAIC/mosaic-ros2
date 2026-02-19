//
// Created by yhkim on 2/8/26.
//

#include "mosaic-ros2-geometry/converter.hpp"

namespace mosaic::ros2::geometry_connector {
geometry_msgs::msg::Vector3 JsonToVector3(Json::Value json) {
    geometry_msgs::msg::Vector3 vector_msg;
    if (json["x"]) {
        vector_msg.x = json["x"].asDouble();
    }
    if (json["y"]) {
        vector_msg.y = json["y"].asDouble();
    }
    if (json["z"]) {
        vector_msg.z = json["z"].asDouble();
    }
    return vector_msg;
}

Json::Value PointToJson(geometry_msgs::msg::Point point) {
    Json::Value json;
    json["x"] = point.x;
    json["y"] = point.y;
    json["z"] = point.z;
    return json;
}

Json::Value QuaternionToJson(geometry_msgs::msg::Quaternion quaternion) {
    Json::Value json;
    json["x"] = quaternion.x;
    json["y"] = quaternion.y;
    json["z"] = quaternion.z;
    json["w"] = quaternion.w;
    return json;
}
}  // namespace mosaic::ros2::geometry_connector