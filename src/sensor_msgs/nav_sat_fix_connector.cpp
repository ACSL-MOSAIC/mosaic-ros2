//
// Created by yhkim on 1/5/26.
//

#include <json/json.h>
#include <mosaic_ros2/sensor_msgs/nav_sat_fix_connector.h>

using namespace mosaic::ros2::sensor_connector;

void NavSatFixConnectorConfigurer::Configure(const std::shared_ptr<core::MosaicConnector> mosaic_connector) {
    MOSAIC_LOG_INFO("Configuring ROS2 sensor_msgs::NavSatFix Connector...");

    handler_ = std::make_shared<NavSatFixDataChannel>(connector_config_.label, mosaic_node_);
    mosaic_connector->AddDataChannelHandler(handler_);

    subscription_ = mosaic_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        connector_config_.params.at("topic_name"),
        10,
        std::bind(&NavSatFixConnectorConfigurer::Callback, this, std::placeholders::_1));

    mosaic_node_->AddSubscription(subscription_);
}

void NavSatFixConnectorConfigurer::Callback(sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (handler_) {
        if (const auto nav_sat_fix_handler = std::dynamic_pointer_cast<NavSatFixDataChannel>(handler_)) {
            nav_sat_fix_handler->OnNavSatFixReceived(msg);
        }
    }
}

void NavSatFixDataChannel::OnNavSatFixReceived(const sensor_msgs::msg::NavSatFix::SharedPtr& nav_sat_fix) {
    if (!Sendable()) {
        return;
    }

    const auto now = std::chrono::high_resolution_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

    Json::Value coordinate_json;

    coordinate_json["latitude"] = nav_sat_fix->latitude;
    coordinate_json["longitude"] = nav_sat_fix->longitude;
    coordinate_json["altitude"] = nav_sat_fix->altitude;
    coordinate_json["timestamp"] = timestamp;

    SendJson(coordinate_json);
}
