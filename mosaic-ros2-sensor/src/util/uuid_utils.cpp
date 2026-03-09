//
// Created by yhkim on 3/9/26.
//

#include "uuid_utils.hpp"
#include <uuid/uuid.h>

namespace mosaic::ros2::sensor_connector::utils {
    std::string GenerateUUID() {
        uuid_t uuid;
        uuid_generate_random(uuid);
        char uuid_str[37];
        uuid_unparse_lower(uuid, uuid_str);
        return std::string(uuid_str);
    }
} // namespace mosaic::ros2::sensor_connector::utils
