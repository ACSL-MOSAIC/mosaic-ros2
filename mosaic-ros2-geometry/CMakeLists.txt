cmake_minimum_required(VERSION 3.8)
project(mosaic-ros2-geometry VERSION 1.0.0)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mosaic-core REQUIRED)
find_package(mosaic-ros2-base REQUIRED)

add_library(mosaic-ros2-geometry STATIC
        src/register.cpp
        src/connector/pose_connector.cpp
        src/connector/twist_connector.cpp
        src/connector/twist_stamped_connector.cpp
        src/converter.cpp
)
target_include_directories(mosaic-ros2-geometry PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mosaic-ros2-geometry
        rclcpp
        geometry_msgs
        mosaic-core
        mosaic-ros2-base
)

install(TARGETS mosaic-ros2-geometry
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(mosaic-ros2-base geometry_msgs)
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
