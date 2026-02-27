# Common ROS Noetic Docker

基于 Docker 的 ROS Noetic 开发环境，预配置了常用工具与依赖。

## 项目简介

本项目提供一个容器化的 ROS Noetic 开发环境，预装了常用的 ROS 功能包、调试工具和开发 utilities。

> **其他语言**: [English](README.md)

## 主要特性

- **基础镜像**: OSRF ROS Noetic desktop full
- **开发工具**: catkin_tools, rosinstall, wstool, build-essential, cmake
- **调试工具**: rqt 套件、RViz、PlotJuggler
- **导航与 SLAM**: navigation 导航栈、gmapping、hector_mapping、robot_localization
- **运动规划**: MoveIt
- **仿真环境**: Gazebo、ros_control
- **通信工具**: rosbridge_server、tf2、actionlib
- **硬件接口**: serial、joy、teleop 系列包
- **数据处理**: sensor_msgs、geometry_msgs、nav_msgs、message_filters

## 快速开始

### 构建镜像

```bash
docker-compose build
```

### 启动容器

```bash
docker-compose up -d
```

### 进入容器

```bash
docker exec -it ros1_noetic_dev bash
```

## 配置说明

### 环境变量

| 变量名 | 默认值 | 说明 |
|--------|--------|------|
| `ROS_IP` | `127.0.0.1` | ROS 网络接口地址 |
| `ROS_MASTER_URI` | `http://localhost:11311` | ROS 主节点 URI |
| `DISPLAY` | - | X11 显示，用于 GUI 应用 |

### 卷映射

- `/tmp/.X11-unix`: X11 套接字，支持图形界面应用
- `/home/hyd`: 用户主目录映射

## 可用工具

### ROS 调试工具
- `rqt-*` - 完整的 rqt 插件套件
- `rviz` - 3D 可视化工具
- `plotjuggler` - 时序数据可视化工具

### 导航与建图
- `robot_localization` - 多传感器状态估计
- `navigation` - 导航功能栈
- `slam_gmapping` - 栅格地图 SLAM
- `hector_mapping` - Hector SLAM 算法

### 机器人控制
- `moveit` - 运动规划框架
- `gazebo_ros_pkgs` - 机器人仿真环境
- `ros_control` - 机器人控制框架

## 依赖要求

- Docker
- Docker Compose
- X11 服务器（用于图形界面应用）

## 许可证

MIT License

## 致谢

- [OSRF](https://www.osrfoundation.org/) - ROS Noetic 基础镜像
- [ROS](https://www.ros.org/) - 机器人操作系统
