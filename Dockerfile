FROM docker.io/library/ros:jazzy

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


RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-dev \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-pip \
    python3-matplotlib \
    python3-lxml \
    python3-pygame \
    python3-numpy \
    python3-serial \
    python3-future \
    python3-lxml \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

RUN pip install mavproxy --break-system-packages

ARG PAUL_repo_DIR="/ws/src/Distributional_RL_Decision_and_Control"

RUN git clone \
    --depth=1 \
    --single-branch \
    --recursive \
    "https://github.com/pszenher/Distributional_RL_Decision_and_Control.git" \
    -b jazzy-vrx-v3.0.3 \
    "${PAUL_repo_DIR}"

RUN apt update

RUN cd ws/ && rosdep install --from-paths src -y --ignore-src -r

RUN . /opt/ros/jazzy/setup.sh && \
    cd ws/ && \
    colcon build \
    --event-handlers "console_direct+" \
    --merge-install \
    --symlink-install
