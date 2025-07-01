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
