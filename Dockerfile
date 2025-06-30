FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# Workaround from https://discourse.ros.org/t/ros-signing-key-migration-guide/43937
RUN rm -f \
    /etc/apt/sources.list.d/ros2-latest.list \
    /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# Only install ros2-apt-source if it's not already installed
RUN if ! dpkg -l | grep -q ros2-apt-source; then \
    apt-get update && \
    apt-get install -y ca-certificates curl && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    apt-get update && \
    apt-get install /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb; \
  fi


# TEMPLATE: Install arbitrary packages
# ================================================================
RUN apt update \
    && apt install -y --no-install-recommends \
       unzip \
       less \
       zip \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean -qq
# ================================================================
# END TEMPLATE

# TEMPLATE: Install Gazebo from package server
# ================================================================
# OPTION #1: Just install the version that ships with ROS Jazzy
# ----------------------------------------------------------------
RUN apt update \
    && apt install -y --no-install-recommends \
       ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean -qq

# OPTION #2: OSRF upstream gazebo package repos
# ----------------------------------------------------------------
# RUN curl -s "https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg" \
#     && \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
#       | tee /etc/apt/sources.list.d/gazebo-stable.list \
#       > /dev/null

# RUN --mount=type=cache,target=/var/cache/apt \
#     apt update \
#     && apt install -y --no-install-recommends \
#        gz-harmonic \
#     && rm -rf /var/lib/apt/lists/* \
#     && apt clean -qq
# ================================================================
# END TEMPLATE


# TEMPLATE: add whatever codebase the dockerfile is in to workspace
# ================================================================

# # Copy workspace files to `/ws/src`
# COPY . /ws/src/

# # Install Deps
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
#     && apt update \
#     && rosdep install -r \
#       --from-paths /ws/src/ \
#       --ignore-src \
#       --rosdistro ${ROS_DISTRO} -y

# # Build the project
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
#     && cd /ws \
#     && colcon build --symlink-install --merge-install
# ================================================================
# END TEMPLATE

RUN apt update && apt install -y sudo python3-pexpect

# RUN apt install python3-pexpect -y

# Create user and set password
RUN useradd -m -s /bin/bash devuser && \
    echo 'devuser:devpass' | chpasswd

# Add user to multiple groups
RUN usermod -aG adm,dialout,cdrom,sudo,dip,plugdev,users devuser

# Optional: Set working directory
WORKDIR /home/devuser

# Switch to the non-root user
USER devuser

RUN mkdir src

WORKDIR /home/devuser/src

RUN git clone --recurse-submodules https://github.com/ardupilot/ardupilot

WORKDIR /home/devuser/

RUN mkdir -p ardu_ws/src

COPY ros2_ardu.repos /home/devuser

RUN vcs import --input ros2_ardu.repos --recursive /home/devuser/ardu_ws/src

RUN rosdep update

USER root

RUN rosdep install -r --from-paths /home/devuser/ardu_ws/src --ignore-src --rosdistro "${ROS_DISTRO}" -y

# RUN curl -s "https://get.sdkman.io" | bash

# RUN sdk install gradle 8.14.2

# RUN bash -c "source $HOME/.sdkman/bin/sdkman-init.sh && sdk install gradle 8.14.2"

RUN apt install openjdk-17-jdk -y

WORKDIR /home/devuser/src

RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Gen.git

WORKDIR /home/devuser/src/Micro-XRCE-DDS-Gen

RUN git config --global --add safe.directory /home/devuser/src/Micro-XRCE-DDS-Gen

RUN git submodule init

RUN git submodule update

RUN ./gradlew assemble

ENV PATH="/home/devuser/src/Micro-XRCE-DDS-Gen/scripts/:$PATH"

WORKDIR /home/devuser/ardu_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/devuser/.bashrc

USER devuser

# RUN colcon build --packages-up-to ardupilot_sitl