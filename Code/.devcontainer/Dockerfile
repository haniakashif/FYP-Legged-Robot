# Start from the official ROS Humble image
FROM osrf/ros:humble-desktop-full

# Non-interactive mode for apt-get
ENV DEBIAN_FRONTEND=noninteractive

# Basic tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    wget \
    lsb-release \
    gnupg \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# ROS-Gazebo Bridge (Harmonic Version)
# We set the environment variable so ROS knows we are using Harmonic
ENV GZ_VERSION=harmonic
RUN apt-get update --fix-missing \ 
&& apt-get remove -y ros-humble-ros-gz* \
&& apt-get install -y \
    ros-humble-ros-gzharmonic \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir \
    onnxruntime \
    "numpy<2.0.0" \
    osqp \
    scipy \
    pin

# ros2_control and xacro
RUN apt-get update --fix-missing && apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# gz_ros2_control (building from source), required for Humble + Harmonic compatibility
# Built into an underlay /opt/ workspace to prevent accidental overwriting by team mounts
RUN apt-get update --fix-missing \
    && mkdir -p /opt/gz_control_ws/src \
    && cd /opt/gz_control_ws/src \
    && git clone -b humble https://github.com/ros-controls/gz_ros2_control.git \
    && cd /opt/gz_control_ws \
    && rosdep init || true \
    && rosdep update \
    && rosdep install -r --from-paths src -i -y --rosdistro humble --skip-keys="ros_gz_sim ros_gz_bridge ros_gz_interfaces gz-plugin2" \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build" \
    && rm -rf /var/lib/apt/lists/*

# user setup
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Non-root user setup with sudo privileges, add to video/render groups for GUI access (for Gazebo and RViz) 
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && (getent group render || groupadd -r render) \
    && usermod -aG video,render $USERNAME

# Source ROS and gz_ros2_control workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /opt/gz_control_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
    
# RViz2
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-visualization-msgs \
    ros-humble-geometry-msgs \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*
    
# Perception dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-grid-map-ros \
    ros-humble-grid-map-msgs \
    ros-humble-grid-map-rviz-plugin \
    ros-humble-grid-map-core \
    ros-humble-grid-map-cv \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-eigen \
    ros-humble-message-filters \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Kindr and Kindr_ros (required by elevation_mapping) added to a separate underlay
RUN mkdir -p /opt/kindr_ws/src && cd /opt/kindr_ws/src \
    && git clone https://github.com/ANYbotics/kindr.git \
    && git clone -b ros2 https://github.com/SivertHavso/kindr_ros.git \
    && cd /opt/kindr_ws \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DUSE_CMAKE=true"

# Source into Kinder underlay
RUN echo "source /opt/kindr_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# Switch user
USER $USERNAME
WORKDIR /home/$USERNAME/ws
