FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    git \
    cmake \
    python3-pip \
    libboost-all-dev \
    libceres-dev \
    libeigen3-dev \
    libpcl-dev \
    nlohmann-json3-dev \
    libusb-1.0-0-dev \
    tmux \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros/ubuntu focal main" \
> /etc/apt/sources.list.d/ros1.list

RUN apt-get update && \
    apt-get install -y --fix-missing && \
    apt-get -f install -y

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    rm -rf build && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

WORKDIR /tmp

RUN git clone --branch 4.2 https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    mkdir build && \
    cd build && \
    cmake .. \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/gtsam

WORKDIR /ros_ws

COPY ./src ./src

# NV-LIOM reads its IMU from /livox/imu_ms2, published by the benchmark's
# imu_g_to_ms2 relay (nv-liom-to-hdmapping package, started by
# nv_liom_benchmark.launch): the Bunker DVI IMU reports acceleration in g and
# NV-LIOM expects m/s^2.
RUN sed -i \
    -e 's|/ouster/points|/livox/pointcloud|g' \
    -e 's|/ouster/imu|/livox/imu_ms2|g' \
    src/nv_liom/config/params_os1_64.yaml && \
    grep -q 'imuTopic: "/livox/imu_ms2"' src/nv_liom/config/params_os1_64.yaml

RUN source /opt/ros/noetic/setup.bash && \
    catkin_make

ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

CMD ["bash"]
