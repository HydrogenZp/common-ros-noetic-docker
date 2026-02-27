# Common ROS Noetic Docker

A Docker-based development environment for ROS Noetic with pre-configured tools and dependencies.

## Overview

This project provides a containerized ROS Noetic development environment with commonly used packages, debugging tools, and development utilities pre-installed.

> **Other Languages**: [简体中文](README_zh.md)

## Features

- **Base Image**: OSRF ROS Noetic desktop full
- **Development Tools**: catkin_tools, rosinstall, wstool, build-essential, cmake
- **Debugging Tools**: rqt suite, RViz, PlotJuggler
- **Navigation & SLAM**: navigation stack, gmapping, hector_mapping, robot_localization
- **Motion Planning**: MoveIt
- **Simulation**: Gazebo, ros_control
- **Communication**: rosbridge_server, tf2, actionlib
- **Hardware Interface**: serial, joy, teleop packages
- **Data Processing**: sensor_msgs, geometry_msgs, nav_msgs, message_filters

## Quick Start

### Build the Image

```bash
docker-compose build
```

### Run the Container

```bash
docker-compose up -d
```

### Access the Container

```bash
docker exec -it ros1_noetic_dev bash
```

> **Note**: Before using, update the username in `docker-compose.yml` from `/home/hyd` to your own home directory path.

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_IP` | `127.0.0.1` | ROS network interface |
| `ROS_MASTER_URI` | `http://localhost:11311` | ROS master URI |
| `DISPLAY` | - | X11 display for GUI applications |

### Volumes

- `/tmp/.X11-unix`: X11 socket for GUI support
- `/home/hyd`: User home directory mapping

## Available Tools

### ROS Debugging
- `rqt-*` - Complete rqt plugin suite
- `rviz` - 3D visualization
- `plotjuggler` - Time series data visualization

### Navigation & Mapping
- `robot_localization` - Multi-sensor state estimation
- `navigation` - Navigation stack
- `slam_gmapping` - Grid-based SLAM
- `hector_mapping` - Hector SLAM

### Robot Control
- `moveit` - Motion planning framework
- `gazebo_ros_pkgs` - Robot simulation
- `ros_control` - Robot control framework

## Dependencies

- Docker
- Docker Compose
- X11 server (for GUI applications)

## License

MIT License

## Acknowledgments

- [OSRF](https://www.osrfoundation.org/) - ROS Noetic base image
- [ROS](https://www.ros.org/) - Robot Operating System
