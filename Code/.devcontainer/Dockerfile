FROM osrf/ros:humble-desktop-full

# Set non-interactive mode for apt-get
ENV DEBIAN_FRONTEND=noninteractive

# 1. Install basic tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    wget \
    lsb-release \
    gnupg \
    curl \
    && rm -rf /var/lib/apt/lists/*

# 2. Install Gazebo Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# 3. Install the ROS-Gazebo Bridge (ONLY Harmonic version)
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
    scipy \
    pin

# 4. Install core ROS 2 Control libraries and URDF tools
RUN apt-get update --fix-missing && apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# 5. Build gz_ros2_control from source (Required for Humble + Harmonic compatibility)
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

# 6. Create the 'ros' user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Fix: Create 'render' group if it doesn't exist, then add user to video/render
    && (getent group render || groupadd -r render) \
    && usermod -aG video,render $USERNAME

# 7. Auto-source core ROS setup AND the underlay control workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /opt/gz_control_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
    
    
    
# Switch user
USER $USERNAME
WORKDIR /home/$USERNAME/ws
