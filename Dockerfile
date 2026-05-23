FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics,display
ENV QT_X11_NO_MITSHM=1

# 🔧 Base system setup
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release sudo locales tzdata \
    build-essential cmake git wget nano \
    python3-pip \
    x11-xserver-utils mesa-utils \
    libeigen3-dev libtinyxml2-dev libgts-dev \
    qtbase5-dev pkg-config libgl1-mesa-glx libglfw3 \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

# Colcon build system
RUN pip3 install -U colcon-common-extensions

# 🧰 ROS 2 Iron setup
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y ros-iron-desktop-full \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install -U rosdep

# 🛠 Init rosdep
RUN rosdep init && rosdep update

# ✅ Install dependencies for Gazebo Fortress + ROS 2
RUN apt-get update && apt-get install -y \
    ros-iron-gazebo-ros-pkgs \
    ros-iron-gazebo-plugins \
    ros-iron-diff-drive-controller \
    ros-iron-joint-state-broadcaster \
    ros-iron-robot-state-publisher \
    ros-iron-xacro \
    ros-iron-ros2-control \
    ros-iron-controller-manager \
    ros-iron-slam-toolbox \
    ros-iron-pointcloud-to-laserscan \
    ros-iron-robot-localization \
    ros-iron-twist-mux \
    ros-iron-tf-transformations \
    lsb-release net-tools \
 && rm -rf /var/lib/apt/lists/*

# 📦 Gazebo Fortress full stack
RUN apt-get update && apt-get install -y \
    libignition-gazebo6-dev \
    libignition-gazebo6-plugins \
    libignition-rendering6-dev \
    libignition-physics5-dev \
    libignition-sensors6-dev \
    libignition-msgs8-dev \
    libignition-common4-dev \
    libignition-utils1-dev \
    libignition-transport11-dev \
    libsdformat12-dev \
 && rm -rf /var/lib/apt/lists/*

# 📦 Project-specific dependencies (from imports)
RUN apt-get update && apt-get install -y \
    ros-iron-geometry-msgs \
    ros-iron-nav-msgs \
    ros-iron-std-msgs \
    ros-iron-sensor-msgs \
    ros-iron-nav2-msgs \
    ros-iron-tf2-ros \
    ros-iron-tf2-geometry-msgs \
    ros-iron-launch \
    ros-iron-launch-ros \
    ros-iron-nav2-common \
    python3-dev \
    python3-tk \
    pyqt5-dev-tools \
    libompl-dev \
    libxtensor-dev \
    pybind11-dev \
 && rm -rf /var/lib/apt/lists/*

# 🔧 Set environment
RUN echo "source /opt/ros/iron/setup.bash" >> /root/.bashrc

# 📦 Python dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

