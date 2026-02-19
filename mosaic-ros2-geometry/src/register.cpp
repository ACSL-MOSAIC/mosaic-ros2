//
// Created by yhkim on 1/20/26.
//

#include "mosaic-ros2-geometry/register.hpp"

#include <mosaic/auto_configurer/connector/connector_resolver.hpp>

#include "mosaic-ros2-geometry/connector/pose_connector.hpp"
#include "mosaic-ros2-geometry/connector/twist_connector.hpp"
#include "mosaic-ros2-geometry/connector/twist_stamped_connector.hpp"

namespace mosaic::ros2::geometry_connector {
    void RegisterConnectors() {
        auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<PoseConnectorConfigurer>();
        auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<TwistConnectorConfigurer>();
        auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<
            TwistStampedConnectorConfigurer>();
    }
} // namespace mosaic::ros2::geometry_connector
