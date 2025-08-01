FROM ros:humble AS base
ARG DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
COPY ../scripts/build/ /tmp/
COPY ../scripts/entrypoint/ros_entrypoint.sh /ros_entrypoint.sh
RUN chown root:root /ros_entrypoint.sh && \
    chmod 755 /ros_entrypoint.sh && \
    sh /tmp/install_depend.sh

FROM base AS librealsense-builder
ARG LIBREALSENSE_VERSION=2.55.1
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    cmake \
    pkg-config \
    build-essential \
    python3 \
    python3-dev \
    ca-certificates \
    libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*
WORKDIR /usr/src
RUN curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$LIBREALSENSE_VERSION -o librealsense.tar.gz && \
    tar -zxf librealsense.tar.gz && \
    rm librealsense.tar.gz && \
    ln -s /usr/src/librealsense-$LIBREALSENSE_VERSION /usr/src/librealsense
# Build librealsense from source
RUN cd /usr/src/librealsense \
    && mkdir build && cd build \
    && cmake \
    -DPYTHON_EXECUTABLE=$(which python3) \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DCMAKE_BUILD_TYPE=Release ../ \
    && make -j$(($(nproc)-1)) all \
    && make install 

FROM base AS realsense
ARG REALSENSE_ROS_VERSION=4.55.1
ARG USER=camera
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_WS_PATH=/home/$USER/vision-ws
# Copy binaries from builder stage
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib
# Install dependencies
RUN apt-get update && apt-get install -y \
    --no-install-recommends \
    libusb-1.0-0 \
    udev \
    ca-certificates \
    ros-humble-diagnostic-updater \
    ros-humble-xacro \
    python3-tqdm \
    python3-requests \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*
RUN sh /tmp/setup_user.sh $USER $USER_UID $USER_GID
ENTRYPOINT [ "/ros_entrypoint.sh" ]
USER $USER
RUN mkdir -p $ROS_WS_PATH/src && \
    # Install ROS2 Realsense package
    git clone --branch $REALSENSE_ROS_VERSION \
        https://github.com/IntelRealSense/realsense-ros.git \
        $ROS_WS_PATH/src/realsense-ros
RUN sh /tmp/rosdep_init.sh $USER
WORKDIR $ROS_WS_PATH

FROM base AS gui
ARG USER=gui
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    ros-humble-rqt* \
    ros-humble-rviz2 \
    ros-humble-foxglove-bridge \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*
RUN sh /tmp/setup_user.sh $USER $USER_UID $USER_GID
ENTRYPOINT [ "/ros_entrypoint.sh" ]
USER $USER
WORKDIR /home/$USER

FROM base AS opencv
ARG USER=gui
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_WS_PATH=/home/$USER/vision-ws
RUN sh /tmp/setup_user.sh $USER $USER_UID $USER_GID
ENTRYPOINT [ "/ros_entrypoint.sh" ]
USER $USER
RUN mkdir -p $ROS_WS_PATH/src && \
    sh /tmp/rosdep_init.sh $USER
WORKDIR $ROS_WS_PATH


