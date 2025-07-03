FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN echo 'APT::Install-Recommends "false";' > "/etc/apt/apt.conf.d/90-docker-no-recommends" \
    && \
    echo 'APT::Install-Suggests "false";'   > "/etc/apt/apt.conf.d/90-docker-no-suggests"

RUN apt update && apt install -y \
      \
      python3-pexpect \
      openjdk-17-jdk \
      ros-jazzy-ros-gz \
      \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean -qq

COPY ros2_ardu.repos "/tmp/ros2_ardu.repos"

RUN mkdir -p "/ws/src" \
    && \
    vcs import \
    --debug \
    --recursive \
    --shallow \
    --input "/tmp/ros2_ardu.repos" \
    "/ws/src"

RUN apt update && \
    rosdep install -r -y \
    --from-paths "/ws/src" \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean -qq

ARG XRCE_DDS_DIR="/ws/deps/micro-xrce-dds-gen"

RUN git clone \
    --depth=1 \
    --single-branch \
    --recursive \
    "https://github.com/ArduPilot/Micro-XRCE-DDS-Gen.git" \
    "${XRCE_DDS_DIR}"

RUN cd "${XRCE_DDS_DIR}" && ./gradlew assemble 

ENV PATH="${XRCE_DDS_DIR}/scripts/:${PATH}"

RUN . /opt/ros/jazzy/setup.sh && \
    cd ws/ && \
    colcon build \
    --event-handlers "console_direct+" \
    --merge-install \
    --symlink-install

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /ws/install/setup.bash" >> /root/.bashrc



# RUN apt update
# RUN sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame -y
# RUN apt install python3.12-venv -y

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    g++ \
    libgtk-3-dev \
    libglib2.0-dev \
    libglu1-mesa-dev \
    libpng-dev \
    libjpeg-dev \
    libtiff-dev \
    libsm6 \
    libxrender1 \
    libxext6 \
    libgl1 \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libnotify-dev \
    freeglut3-dev \
    python3-dev \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-pip \
    python3.12-venv \
    python3-matplotlib \
    python3-lxml \
    python3-pygame \
    pkg-config \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

ENV VENV_PATH="/opt/myvenv"

RUN python3 -m venv $VENV_PATH && \
    $VENV_PATH/bin/pip install --upgrade pip && \
    $VENV_PATH/bin/pip install PyYAML mavproxy pexpect empy==3.3.4 opencv-python wxPython matplotlib


RUN echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
