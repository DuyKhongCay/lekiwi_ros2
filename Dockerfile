# syntax=docker/dockerfile:1
# ==============================================================================
# LeKiwi ROS 2 Robot - Core Runtime & Vision Environment (Raspberry Pi 5 / Hailo-8)
# Target Architecture: linux/arm64
# Base: ROS 2 Jazzy (Ubuntu 24.04 Noble) + HailoRT 4.23.0 + TAPPAS 5.1.0
# ==============================================================================

FROM ros:jazzy-ros-base

LABEL org.opencontainers.image.title="lekiwi-ros2" \
      org.opencontainers.image.description="LeKiwi ROS 2 Jazzy container with Raspberry Pi 5 CSI camera, Hailo-8 NPU, and Zenoh middleware" \
      org.opencontainers.image.version="jazzy-hailort4.23-tappas5.1" \
      org.opencontainers.image.vendor="LeKiwi Robotics" \
      org.opencontainers.image.licenses="Apache-2.0"

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# ------------------------------------------------------------------------------
# 1. System locales & Python 3.12 default alternatives
# ------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales \
        python3.12 \
        python3.12-dev \
        python3-pip \
        python3-setuptools \
        python3-wheel \
    && locale-gen en_US.UTF-8 \
    && update-alternatives --install /usr/bin/python python /usr/bin/python3.12 1 \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.12 1 \
    && update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 1 \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# 2. System Utilities, Media, OpenCV, Qt6, GStreamer & ROS 2 Middleware
# ------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        gnupg \
        ca-certificates \
        git \
        build-essential \
        cmake \
        # Media & Vision
        ffmpeg \
        x11-utils \
        libopencv-dev \
        python3-opencv \
        # Qt GUI & XCB platform dependencies
        libxcb-xinerama0 \
        libxcb-icccm4 \
        libxcb-image0 \
        libxcb-keysyms1 \
        libxcb-render-util0 \
        libxcb-shape0 \
        libxcb-randr0 \
        libxcb-cursor0 \
        libxkbcommon-x11-0 \
        libdbus-1-3 \
        libgl1-mesa-dri \
        libgl1 \
        libglx-mesa0 \
        qt6-base-dev \
        # GStreamer framework & development headers
        libgstreamer1.0-0 \
        libgstreamer-plugins-base1.0-0 \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        gstreamer1.0-tools \
        # Python GObject introspection & Cairo
        python3-gi \
        python3-gi-cairo \
        gir1.2-gtk-3.0 \
        libgirepository1.0-dev \
        libcairo2-dev \
        # ROS 2 Vision and Middleware packages
        ros-jazzy-cv-bridge \
        ros-jazzy-vision-msgs \
        ros-jazzy-image-transport \
        ros-jazzy-rmw-zenoh-cpp \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-xacro \
        # Boost and TFLite dependencies for libcamera 0.7 / PiSP
        libboost-filesystem1.83.0 \
        libboost-thread1.83.0 \
        libboost-log1.83.0 \
        libcpuinfo0 \
        libfarmhash0 \
        # Hailo / TAPPAS runtime dependencies
        libzmq3-dev \
        libgomp1 \
        libgoogle-glog-dev \
        pciutils \
        # gst-shark & profiling dependencies
        autoconf \
        automake \
        libtool \
        graphviz \
        libgraphviz-dev \
        gtk-doc-tools \
        libbabeltrace-dev \
        zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# 3. Raspberry Pi 5 official libcamera 0.7 & libcamerasrc
# ------------------------------------------------------------------------------
COPY docker_packages/gstreamer/*.deb /tmp/libcamera_packages/
RUN dpkg -i /tmp/libcamera_packages/*.deb \
    && rm -rf /tmp/libcamera_packages \
    # Patch GStreamer minor version from 1.26 (\x1a) to 1.24 (\x18) for Ubuntu 24.04 Noble compatibility
    && sed -i 's/\x01\x00\x00\x00\x1a\x00\x00\x00/\x01\x00\x00\x00\x18\x00\x00\x00/g' /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstlibcamera.so 2>/dev/null || true

# ------------------------------------------------------------------------------
# 4. HailoRT 4.23.0 C/C++ runtime & Python 3.12 bindings
# ------------------------------------------------------------------------------
COPY docker_packages/hailo/hailort_4.23.0_arm64.deb /tmp/
COPY docker_packages/hailo/hailort-4.23.0-cp312-cp312-linux_aarch64.whl /tmp/
RUN mkdir -p /tmp/hailo_root \
    && dpkg-deb -x /tmp/hailort_4.23.0_arm64.deb /tmp/hailo_root \
    && [ -d /tmp/hailo_root/usr/bin ] && cp -a /tmp/hailo_root/usr/bin/. /usr/bin/ || true \
    && [ -d /tmp/hailo_root/usr/lib ] && cp -a /tmp/hailo_root/usr/lib/. /usr/lib/ || true \
    && [ -d /tmp/hailo_root/usr/include ] && cp -a /tmp/hailo_root/usr/include/. /usr/include/ || true \
    && [ -d /tmp/hailo_root/usr/share ] && cp -a /tmp/hailo_root/usr/share/. /usr/share/ || true \
    && rm -rf /tmp/hailo_root /tmp/hailort_4.23.0_arm64.deb \
    && python3.12 -m pip install --no-cache-dir --break-system-packages /tmp/hailort-4.23.0-cp312-cp312-linux_aarch64.whl \
    && rm -f /tmp/hailort-4.23.0-cp312-cp312-linux_aarch64.whl

# ------------------------------------------------------------------------------
# 5. TAPPAS Core 5.1.0 runtime & Python bindings
# ------------------------------------------------------------------------------
COPY docker_packages/hailo/hailo-tappas-core_5.1.0_arm64.deb /tmp/
COPY docker_packages/hailo/hailo_tappas_core_python_binding-5.1.0-py3-none-any.whl /tmp/
RUN mkdir -p /tmp/tappas_root \
    && dpkg-deb -x /tmp/hailo-tappas-core_5.1.0_arm64.deb /tmp/tappas_root \
    && [ -d /tmp/tappas_root/usr/bin ] && cp -a /tmp/tappas_root/usr/bin/. /usr/bin/ || true \
    && [ -d /tmp/tappas_root/usr/lib ] && cp -a /tmp/tappas_root/usr/lib/. /usr/lib/ || true \
    && [ -d /tmp/tappas_root/usr/include ] && cp -a /tmp/tappas_root/usr/include/. /usr/include/ || true \
    && [ -d /tmp/tappas_root/usr/share ] && cp -a /tmp/tappas_root/usr/share/. /usr/share/ || true \
    && mkdir -p /usr/include/hailo/tappas/gsthailometa \
                /usr/include/hailo/tappas/general \
                /usr/include/hailo/tappas/plugins/common \
                /usr/include/hailo/tappas/tracking \
                /usr/include/gstreamer-1.0/gst/hailo \
    # Symlink OpenCV 4.10 names required by TAPPAS 5.1.0 to system OpenCV 4.6.0
    && ln -sf /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.6.0 /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.410 \
    && ln -sf /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.6.0 /usr/lib/aarch64-linux-gnu/libopencv_core.so.410 \
    # Patch GStreamer minor version from 1.26 (\x1a) to 1.24 (\x18)
    && sed -i 's/\x01\x00\x00\x00\x1a\x00\x00\x00/\x01\x00\x00\x00\x18\x00\x00\x00/g' /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgsthailotools.so.5.1.0 2>/dev/null || true \
    && sed -i 's/\x01\x00\x00\x00\x1a\x00\x00\x00/\x01\x00\x00\x00\x18\x00\x00\x00/g' /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgsthailotracers.so.5.1.0 2>/dev/null || true \
    && rm -rf /tmp/tappas_root /tmp/hailo-tappas-core_5.1.0_arm64.deb \
    && python3.12 -m pip install --no-cache-dir --break-system-packages /tmp/hailo_tappas_core_python_binding-5.1.0-py3-none-any.whl \
    && rm -f /tmp/hailo_tappas_core_python_binding-5.1.0-py3-none-any.whl

# ------------------------------------------------------------------------------
# 6. Profiling Tools (gst-shark & ctf2ctf)
# ------------------------------------------------------------------------------
RUN cd /tmp \
    && git clone --depth 1 https://github.com/RidgeRun/gst-shark.git \
    && cd gst-shark \
    && ./autogen.sh --prefix=/usr --libdir=/usr/lib/aarch64-linux-gnu \
    && make -j$(nproc) \
    && make install \
    && cd /tmp \
    && rm -rf /tmp/gst-shark \
    && git clone --recursive --depth 1 https://github.com/KDABLabs/ctf2ctf.git \
    && cd ctf2ctf \
    && mkdir build && cd build \
    && cmake -DBabeltrace_LIBRARY_DIR=/usr/lib/aarch64-linux-gnu .. \
    && make -j$(nproc) \
    && cp ctf2ctf /usr/local/bin/ \
    && cd /tmp \
    && rm -rf /tmp/ctf2ctf

# ------------------------------------------------------------------------------
# 7. Environment & Runtime Configurations
# ------------------------------------------------------------------------------
ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:/usr/lib:/usr/local/lib \
    LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Configure custom bash prompt & auto-source ROS 2 / workspace install
RUN echo 'export PS1="\[\033[01;35m\]\u@lekiwi-pi5\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /root/.bashrc \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo '[ -f /root/docker_ws/lekiwi_ros2/install/setup.bash ] && source /root/docker_ws/lekiwi_ros2/install/setup.bash' >> /root/.bashrc \
    && echo 'export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:/usr/lib:/usr/local/lib:$LD_LIBRARY_PATH' >> /root/.bashrc

WORKDIR /root/docker_ws/lekiwi_ros2

CMD ["bash"]

