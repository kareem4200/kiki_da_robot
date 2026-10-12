# ros2-humble
FROM ros:humble-ros-base

# ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-navigation2 ros-humble-nav2-bringup \
      ros-humble-robot-localization ros-humble-slam-toolbox \
      ros-humble-cartographer-ros \
      ros-humble-rmw-cyclonedds-cpp ros-humble-demo-nodes-cpp \
      ros-humble-ros2-control ros-humble-ros2-controllers \
      ros-humble-controller-manager ros-humble-hardware-interface \
      ros-humble-diff-drive-controller ros-humble-joint-state-broadcaster \
      ros-humble-ros2controlcli ros-humble-ros2-controllers-test-nodes \
      ros-humble-xacro ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher ros-humble-rviz2 \
      python3-colcon-common-extensions python3-rosdep python3-pip \
      cmake build-essential git libeigen3-dev \
      i2c-tools usbutils \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir smbus2

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# YDLidar SDK
RUN git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git /opt/YDLidar-SDK \
    && mkdir /opt/YDLidar-SDK/build && cd /opt/YDLidar-SDK/build \
    && cmake .. && make -j2 && make install

# JetsonGPIO needed by kiki_control
ARG JETSON_GPIO_REF=master
RUN git clone https://github.com/pjueon/JetsonGPIO.git /tmp/JetsonGPIO \
    && cd /tmp/JetsonGPIO && git checkout ${JETSON_GPIO_REF} \
    && cmake -S . -B build \
         -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_EXAMPLES=OFF \
         -DJETSON_GPIO_POST_INSTALL=OFF \
    && cmake --build build --target install -j2 \
    && ldconfig \
    && rm -rf /tmp/JetsonGPIO
