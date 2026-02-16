//
// Created by yhkim on 1/20/26.
//

#include "mosaic-ros2-sensor/register.hpp"

#include <mosaic/auto_configurer/connector/connector_resolver.hpp>

#include "mosaic-ros2-sensor/connector/image_connector.hpp"
#include "mosaic-ros2-sensor/connector/nav_sat_fix_connector.hpp"
#include "mosaic-ros2-sensor/connector/point_cloud2_connector.hpp"

namespace mosaic::ros2::sensor_connector {
    void RegisterConnectors() {
        auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<ImageConnectorConfigurer>();
        auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<NavSatFixConnectorConfigurer>();
        auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<
            PointCloud2ConnectorConfigurer>();
    }
} // namespace mosaic::ros2::sensor_connector
