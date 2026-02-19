cmake_minimum_required(VERSION 3.8)
project(mosaic-ros2-sensor VERSION 1.0.0)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(mosaic-ros2-base REQUIRED)

find_package(Protobuf REQUIRED)
if (NOT Protobuf_PROTOC_EXECUTABLE)
    message(FATAL_ERROR "protoc executable not found!")
endif ()

find_package(PkgConfig REQUIRED)
pkg_check_modules(UUID REQUIRED uuid)

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

set(PROTO_FILES
        protobuf/pc2_meta.proto
        protobuf/pc2_chunk.proto
        protobuf/pc2_statistic.proto
)

# Set custom output directory for protobuf generated files
set(PROTO_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto_gen)
set(PROTO_INCLUDE_DIR ${PROTO_OUTPUT_DIR})
file(MAKE_DIRECTORY ${PROTO_OUTPUT_DIR})

# Manually generate protobuf files with custom output directory
set(PROTO_SRCS)
set(PROTO_HDRS)
foreach (PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    list(APPEND PROTO_SRCS ${PROTO_OUTPUT_DIR}/protobuf/${PROTO_NAME}.pb.cc)
    list(APPEND PROTO_HDRS ${PROTO_OUTPUT_DIR}/protobuf/${PROTO_NAME}.pb.h)
endforeach ()

# Generate C++ source files from .proto files
add_custom_command(
        OUTPUT ${PROTO_SRCS} ${PROTO_HDRS}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        --cpp_out=${PROTO_OUTPUT_DIR}
        --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
        ${PROTO_FILES}
        DEPENDS ${PROTO_FILES}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating protobuf C++ files in proto_gen"
)

add_library(mosaic-ros2-sensor STATIC
        src/register.cpp
        src/connector/image_connector.cpp
        src/connector/nav_sat_fix_connector.cpp
        src/connector/point_cloud2_connector.cpp
        src/util/point_cloud2_sender.cpp
        ${PROTO_SRCS}
)
target_include_directories(mosaic-ros2-sensor PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_include_directories(mosaic-ros2-sensor PRIVATE
        ${UUID_INCLUDE_DIRS}
        ${PROTO_INCLUDE_DIR}
)
target_link_libraries(mosaic-ros2-sensor ${UUID_LIBRARIES})
ament_target_dependencies(mosaic-ros2-sensor
        rclcpp
        sensor_msgs
        cv_bridge
        Protobuf
        mosaic-ros2-base
)

install(TARGETS mosaic-ros2-sensor
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(mosaic-ros2-base sensor_msgs cv_bridge Protobuf)
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
