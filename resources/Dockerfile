# ROS2 Humble (Ubuntu 22.04 LTS)
FROM osrf/ros:humble-desktop-full

# ── System dependencies ────────────────────────────────────────────────────────
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    # Python dev
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # ROS packages
    ros-humble-ros2-control \
    ros-humble-gz-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-slam-toolbox \
    ros-humble-backward-ros \
    libgflags-dev \
    nlohmann-json3-dev \
    # Handy utilities
    bash-completion \
    curl \
    nano \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ── rosdep init (skip if already done in base image) ──────────────────────────
RUN rosdep update

# ── Auto-source ROS and the local workspace on every shell ────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc \
    && echo 'export LIDAR_TYPE="A1"' >> /root/.bashrc

# ── Default shell entrypoint ──────────────────────────────────────────────────
CMD ["/bin/bash"]
