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
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir \
    onnxruntime

# 4. Create the 'ros' user
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

# 5. Auto-source ROS setup
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Switch user
USER $USERNAME
WORKDIR /home/$USERNAME/ws
