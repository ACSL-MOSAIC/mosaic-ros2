cmake_minimum_required(VERSION 3.8)
project(mosaic-ros2)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mosaic-rtc-core REQUIRED)
find_package(mosaic-auto-configurer REQUIRED)

find_package(OpenSSL REQUIRED COMPONENTS SSL Crypto)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Read ROS_DISTRO environment variable
if (DEFINED ENV{ROS_DISTRO})
    set(ROS_DISTRO $ENV{ROS_DISTRO})
    message(STATUS "Detected ROS_DISTRO: ${ROS_DISTRO}")

    # Define preprocessor macro to distinguish between Humble and Jazzy
    if (ROS_DISTRO STREQUAL "humble")
        add_compile_definitions(ROS_DISTRO_HUMBLE)
    elseif (ROS_DISTRO STREQUAL "jazzy")
        add_compile_definitions(ROS_DISTRO_JAZZY)
    endif ()
else ()
    message(WARNING "ROS_DISTRO environment variable not set")
endif ()

add_library(mosaic_geometry_msgs STATIC
        src/geometry_msgs/twist_connector.cpp
        src/geometry_msgs/twist_stamped_connector.cpp
)
target_include_directories(mosaic_geometry_msgs PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(mosaic_geometry_msgs
        mosaic::mosaic-auto-configurer
)
ament_target_dependencies(mosaic_geometry_msgs
        rclcpp
        geometry_msgs
)

add_library(mosaic_sensor_msgs STATIC
        src/sensor_msgs/image_connector.cpp
        src/sensor_msgs/nav_sat_fix_connector.cpp
        src/sensor_msgs/point_cloud2_connector.cpp
)
target_include_directories(mosaic_sensor_msgs PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(mosaic_sensor_msgs
        mosaic::mosaic-auto-configurer
)
ament_target_dependencies(mosaic_sensor_msgs
        rclcpp
        sensor_msgs
        cv_bridge
)

add_executable(mosaic-ros2
        src/mosaic_ros2_main.cpp
        src/mosaic_node.cpp
        src/ros2_auto_configurer.cpp
        src/ros_logger.cpp
)
target_link_libraries(mosaic-ros2
        mosaic_sensor_msgs
        mosaic_geometry_msgs
        mosaic::mosaic-rtc-core
        mosaic::mosaic-auto-configurer
)

target_include_directories(mosaic-ros2 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(mosaic-ros2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(mosaic-ros2
        rclcpp
)

install(TARGETS mosaic-ros2
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

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
