# MOSAIC-ROS2

ROS2 integration package for [MOSAIC](https://github.com/ACSL-MOSAIC/mosaic-core) (Multi-Objective System for Advanced
Interactive Control), a WebRTC-based robotics communication framework.

## Overview

MOSAIC-ROS2 provides ROS2 nodes and utilities to seamlessly integrate MOSAIC's WebRTC communication capabilities with
ROS2 robotics applications. This allows real-time, low-latency streaming of sensor data (camera, point cloud, GPS) and
geometry messages over WebRTC.

## Package Structure

```
mosaic-ros2/
├── mosaic-ros2-base/       # Core MOSAIC-ROS2 interfaces and base classes
├── mosaic-ros2-geometry/   # Geometry message connectors (Twist, Pose, etc.)
├── mosaic-ros2-sensor/     # Sensor message connectors (Image, PointCloud2, etc.)
├── mosaic-ros2-bringup/    # Launch files and main executable
└── dockerfiles/            # Docker configurations for Jazzy and Humble
```

### Package Descriptions

- **mosaic-ros2-base**: Provides base classes and utilities for MOSAIC-ROS2 integration with mosaic-core
- **mosaic-ros2-geometry**: Handles ROS2 geometry messages (Twist, Pose, etc.) for WebRTC streaming
- **mosaic-ros2-sensor**: Handles ROS2 sensor messages (Image, PointCloud2, etc.) for WebRTC streaming
- **mosaic-ros2-bringup**: Main launch package to bring up the MOSAIC-ROS2 system

## Requirements

### System Requirements

- Ubuntu 22.04 (for ROS2 Humble) or Ubuntu 24.04 (for ROS2 Jazzy)
- ROS2 Humble or Jazzy
- C++17 compatible compiler

### Dependencies

- [mosaic-core](https://github.com/ACSL-MOSAIC/mosaic-core)
- ROS2 packages: `rclcpp`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`
- System libraries: `libjsoncpp-dev`, `libssl-dev`, `libboost-all-dev`, `libopencv-dev`, `libwebsocketpp-dev`,
  `libcpprest-dev`, `libfmt-dev`, `libprotobuf-dev`, `uuid-dev`

## Building from Source

### 1. Install ROS2

Follow the official ROS2 installation guide:

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

### 2. Install Dependencies

```bash
sudo apt update
sudo apt install -y \
    libjsoncpp-dev libssl-dev libboost-all-dev libopencv-dev \
    libwebsocketpp-dev libcpprest-dev libfmt-dev libprotobuf-dev uuid-dev
```

### 3. Install mosaic-core

```bash
cd ~
git clone https://github.com/ACSL-MOSAIC/mosaic-core.git
cd mosaic-core/third_party
tar -xzvf webrtc-tarballs/webrtc-6723-x86.tar.gz
cd ..
mkdir build && cd build
cmake -G Ninja .. && ninja install
```

### 4. Build MOSAIC-ROS2

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/ACSL-MOSAIC/mosaic-ros2.git

# Build
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Running

### Basic Launch

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mosaic-ros2-bringup mosaic_bringup_launch.py
```

### With Configuration File

```bash
ros2 launch mosaic-ros2-bringup mosaic_bringup_launch.py \
    mosaic_config:=/path/to/your/config.yaml
```

### Launch Arguments

- `mosaic_config`: Path to YAML configuration file (default: `./mosaic_config.yaml`)
- `mosaic_log_level`: MOSAIC library log level - `debug`, `info`, `warning`, `error` (default: `info`)
- `webrtc_log_level`: WebRTC log level - `none`, `verbose`, `info`, `warning`, `error` (default: `none`)

## Docker Usage

Pre-configured Docker images are available for both ROS2 Humble and Jazzy with rosbag playback support.

### Building Docker Images

**For ROS2 Jazzy:**

```bash
cd ~/ros2_ws/src/mosaic-ros2
docker build -f dockerfiles/Dockerfile-jazzy-rosbag-simulator \
    -t mosaic-rosbag-sim:jazzy .
```

**For ROS2 Humble:**

```bash
cd ~/ros2_ws/src/mosaic-ros2
docker build -f dockerfiles/Dockerfile-humble-rosbag-simulator \
    -t mosaic-rosbag-sim:humble .
```

### Running Docker Containers

> **Note**: Configuration file volume mounting is **required** for running MOSAIC-ROS2.

**Basic run (config only):**

```bash
docker run -it \
    -v /path/to/your/config.yaml:/root/mosaic_config/config.yaml \
    mosaic-rosbag-sim:jazzy
```

**With rosbag playback:**

```bash
docker run -it \
    -v /path/to/your/config.yaml:/root/mosaic_config/config.yaml \
    -v /path/to/your/rosbag:/root/rosbag \
    mosaic-rosbag-sim:jazzy
```

**With custom config path (using environment variable):**

```bash
docker run -it \
    -v /path/to/your/config.yaml:/root/config.yaml \
    -e MOSAIC_CONFIG_PATH=/root/config.yaml \
    mosaic-rosbag-sim:jazzy
```

**Recommended: Full example with network host, rosbag and config:**

```bash
docker run -it --network host \
    -v /path/to/your/config.yaml:/root/mosaic_config/config.yaml \
    -v /path/to/your/rosbag:/root/rosbag \
    mosaic-rosbag-sim:jazzy
```

### Docker Features

- **Required config file**: Configuration file must be mounted via volume (default path:
  `/root/mosaic_config/config.yaml` or custom path with `MOSAIC_CONFIG_PATH` environment variable)
- **Automatic rosbag playback**: If rosbag files are mounted to `/root/rosbag`, they will automatically play in loop
  mode
- **Flexible config mounting**: Use default path (`/root/mosaic_config/config.yaml`) or custom path with
  `MOSAIC_CONFIG_PATH` environment variable
- **Multi-ROS2 support**: Same entrypoint script works for both Humble and Jazzy

## License

This project is licensed under the Apache-2.0 License - see the [LICENSE](LICENSE) file for details.

## Maintainer

- **GIST ACSL - Yeonhyuk Kim** - [brankein13@gm.gist.ac.kr](mailto:brankein13@gm.gist.ac.kr)

## Related Projects

- [MOSAIC Core](https://github.com/ACSL-MOSAIC/mosaic-core) - Core WebRTC communication library