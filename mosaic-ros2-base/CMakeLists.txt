cmake_minimum_required(VERSION 3.8)
project(mosaic-ros2-base VERSION 1.0.0)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mosaic-core REQUIRED)

find_package(OpenSSL REQUIRED COMPONENTS SSL Crypto)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(mosaic-ros2-base STATIC
        src/configurer/ros2_auto_configurer.cpp
        src/configurer/ros2_connector_configurer.cpp
        src/node/mosaic_node.cpp
        src/logger/ros_logger.cpp
)
target_link_libraries(mosaic-ros2-base
        mosaic::mosaic-core
)

target_include_directories(mosaic-ros2-base PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(mosaic-ros2-base PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(mosaic-ros2-base
        rclcpp
)

install(TARGETS mosaic-ros2-base
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs mosaic-core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
