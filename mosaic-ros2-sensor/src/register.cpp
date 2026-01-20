//
// Created by yhkim on 1/20/26.
//

#include "mosaic-ros2-sensor/register.h"

#include <mosaic/auto_configurer/connector/connector_resolver.h>

#include "mosaic-ros2-sensor/connector/image_connector.h"
#include "mosaic-ros2-sensor/connector/nav_sat_fix_connector.h"
#include "mosaic-ros2-sensor/connector/point_cloud2_connector.h"

namespace mosaic::ros2::sensor_connector {
void RegisterConnectors() {
    auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<ImageConnectorConfigurer>();
    auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<NavSatFixConnectorConfigurer>();
    auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<PointCloud2ConnectorConfigurer>();
}
}  // namespace mosaic::ros2::sensor_connector
