//
// Created by yhkim on 1/20/26.
//

#include "mosaic-ros2-geometry/register.h"

#include <mosaic/auto_configurer/connector/connector_resolver.h>

#include "mosaic-ros2-geometry/connector/twist_connector.h"
#include "mosaic-ros2-geometry/connector/twist_stamped_connector.h"

namespace mosaic::ros2::geometry_connector {
void RegisterConnectors() {
    auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<TwistConnectorConfigurer>();
    auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<TwistStampedConnectorConfigurer>();
}
}  // namespace mosaic::ros2::geometry_connector
