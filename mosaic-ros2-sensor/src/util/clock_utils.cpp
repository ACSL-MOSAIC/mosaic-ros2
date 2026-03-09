//
// Created by yhkim on 3/9/26.
//

#include "clock_utils.hpp"

#include <chrono>

namespace mosaic::ros2::sensor_connector::utils {
    long GetNow() {
        const auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    }
} // namespace mosaic::ros2::sensor_connector::utils

