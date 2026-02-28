FROM osrf/ros:noetic-desktop-full

USER root

# 设置环境变量，避免交互式提示
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# ROS 环境配置
ENV ROBOT_TYPE=standard6
ENV ROS_IP=127.0.0.1
ENV ROS_MASTER_URI=http://localhost:11311

ARG HOST_HOME_DIR=/home/hyd
ENV HOST_HOME_DIR=${HOST_HOME_DIR}

RUN apt-get update && \
    apt-get install -y sudo software-properties-common wget gnupg curl && \
    # 安装 ROS Noetic 额外常用包
    # ROS 调试工具 - rqt 全套
    apt-get install -y \
        ros-noetic-rqt-* \
        # RViz
        ros-noetic-rviz \
        ros-noetic-rviz-plugin-tutorials \
        ros-noetic-rviz-visual-tools \
        # PlotJuggler
        ros-noetic-plotjuggler \
        ros-noetic-plotjuggler-ros \
        # 功能包
        ros-noetic-robot-localization \
        ros-noetic-navigation \
        ros-noetic-slam-gmapping \
        ros-noetic-hector-mapping \
        ros-noetic-moveit \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-ros-control \
        ros-noetic-ros-controllers \
        ros-noetic-joint-state-publisher \
        ros-noetic-xacro \
        ros-noetic-urdf \
        ros-noetic-geometry-msgs \
        ros-noetic-sensor-msgs \
        ros-noetic-nav-msgs \
        ros-noetic-tf2-ros \
        ros-noetic-tf2-geometry-msgs \
        ros-noetic-message-filters \
        ros-noetic-rosbridge-server \
        ros-noetic-joy \
        ros-noetic-teleop-twist-keyboard \
        ros-noetic-teleop-twist-joy \
        ros-noetic-imu-complementary-filter \
        ros-noetic-imu-filter-madgwick \
        ros-noetic-actionlib \
        ros-noetic-actionlib-msgs \
        ros-noetic-serial \
        ros-noetic-rosmon \
        ros-noetic-rosmon-core \
        ros-noetic-rosmon-msgs \
        # 安装其他依赖
        python3-pip \
        python3-catkin-tools \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential \
        cmake \
        git \
        vim \
        jq \
        libserial-dev && \
    # 安装新版 cmake 和 clangd-21 (从 LLVM 源)
    wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - && \
    apt-add-repository 'deb http://apt.llvm.org/focal/ llvm-toolchain-focal-21 main' && \
    apt-get update && \
    apt-get install -y cmake clangd-21 && \
    apt-get install -y clang-format && \
    ln -s /usr/bin/clangd-21 /usr/local/bin/clangd && \
    # 初始化 rosdep (如果已存在则先删除)
    rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update && \
    # 清理
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 配置颜色化 bash 和 ROS 环境变量
RUN echo '# 颜色化 bash' >> /root/.bashrc && \
    echo 'export TERM=xterm-256color' >> /root/.bashrc && \
    echo "alias ls='ls --color=auto'" >> /root/.bashrc && \
    echo "alias grep='grep --color=auto'" >> /root/.bashrc && \
    echo "alias ll='ls -alF --color=auto'" >> /root/.bashrc && \
    echo "alias pl='rosrun plotjuggler plotjuggler'" >> /root/.bashrc && \
    echo 'PS1="${debian_chroot:+($debian_chroot)}\[\\033[01;32m\]\u@\h\[\\033[00m\]:\[\\033[01;34m\]\w\[\\033[00m\]\$ "' >> /root/.bashrc && \
    echo '' >> /root/.bashrc && \
    echo '# ROS 环境配置' >> /root/.bashrc && \
    echo 'export ROBOT_TYPE=standard6' >> /root/.bashrc && \
    echo 'export ROS_IP=127.0.0.1' >> /root/.bashrc && \
    echo 'export ROS_MASTER_URI=http://localhost:11311' >> /root/.bashrc && \
    echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
WORKDIR ${HOST_HOME_DIR}
LABEL org.opencontainers.image.source=https://github.com/HydrogenZp/common-ros-noetic-docker
LABEL org.opencontainers.image.description="Common ROS Noetic Docker Image with pre-configured tools"
LABEL org.opencontainers.image.licenses=MIT
