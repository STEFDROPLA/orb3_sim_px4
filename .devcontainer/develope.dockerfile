FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Set up locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Install base packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    build-essential \
    wget \
    git \
    cmake \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-yaml \
    python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 Humble repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble and required tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# XRCE-DDS agent from source (instead of package)
WORKDIR /opt
RUN apt-get update && apt-get install -y libasio-dev libtinyxml2-dev && \
    rm -rf Micro-XRCE-DDS-Agent && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig

# PX4 dependencies
RUN apt-get update -y --quiet && \
    DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
    astyle \
    build-essential \
    cmake \
    cppcheck \
    file \
    g++ \
    gcc \
    gdb \
    git \
    lcov \
    libfuse2 \
    libxml2-dev \
    libxml2-utils \
    make \
    ninja-build \
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    rsync \
    shellcheck \
    unzip \
    zip \
    dmidecode \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libeigen3-dev \
    libgstreamer-plugins-base1.0-dev \
    libimage-exiftool-perl \
    libopencv-dev \
    pkg-config \
    protobuf-compiler \
    && pip3 install kconfiglib \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo 11
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    gazebo \
    gazebo-common \
    gazebo-plugin-base \
    libgazebo-dev && \
    rm -rf /var/lib/apt/lists/*


# Install JSBSim
RUN wget https://github.com/JSBSim-Team/jsbsim/releases/download/v1.1.13/JSBSim-devel_1.1.13-986.jammy.amd64.deb && \
    dpkg -i JSBSim-devel_1.1.13-986.jammy.amd64.deb && \
    rm JSBSim-devel_1.1.13-986.jammy.amd64.deb


# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi && \
    apt-get update && apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME


   
RUN sudo usermod --append --groups video $USERNAME
# Set the working directory
#WORKDIR /home/ubuntu

# Source ROS2 setup script by default in the user's .bashrc (fix for point 3)
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc

# Initialize rosdep safely in Docker (fix for point 5)
USER root
RUN rosdep init || true && rosdep update
USER $USERNAME


# # Install OpenCV dependencies and build OpenCV 4.4.0
# USER root
# RUN apt-get update && apt-get install -y \
#      python3-dev python3-numpy python2-dev \
#      libavcodec-dev libavformat-dev libswscale-dev \
#      libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
#      libgtk-3-dev && \
#      cd /tmp && git clone https://github.com/opencv/opencv.git && \
#      cd opencv && git checkout 4.2.0 && mkdir build && cd build && \
#      cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=OFF -D BUILD_DOCS=OFF \
#            -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
#      make -j$(nproc) && make install && \
#      cd / && rm -rf /tmp/opencv
#RUN apt-get update && apt-get install -y ros-humble-vision-opencv

USER root
# Build and install Pangolin v0.9.1
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && git checkout v0.9.1 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14 -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && make install && \
    cd / && rm -rf /tmp/Pangolin
    

USER $USERNAME


CMD ["bash"]