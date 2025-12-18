# syntax=docker/dockerfile:1
# Use OSRF ROS image (has arm64/amd64 variants). Make sure to build for linux/arm64 on Apple Silicon.
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
SHELL ["/bin/bash", "-c"]

# --- OS + ROS deps (include tf_transformations fix) + Gazebo Sim bridge + Nav2 basics ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl ca-certificates \
    python3-pip python3-rosdep python3-vcstool python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    # --- noVNC + lightweight desktop ---
    xvfb x11vnc novnc websockify \
    xfce4 xfce4-terminal dbus-x11 \
    # --- OpenGL software fallback (important on Mac Docker) ---
    mesa-utils libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*

# --- CycloneDDS config (multi-robot friendly) ---
RUN printf '%s\n' \
'<CycloneDDS>' \
'  <Discovery>' \
'    <ParticipantIndex>auto</ParticipantIndex>' \
'    <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>' \
'  </Discovery>' \
'</CycloneDDS>' \
> /etc/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# --- rosdep init/update ---
RUN (rosdep init || true) && rosdep update

# --- Workspace ---
WORKDIR /opt/ros2_ws
RUN mkdir -p src

# tb3_multi_robot (master branch)
RUN git clone --depth 1 https://github.com/arshadlab/tb3_multi_robot.git src/tb3_multi_robot -b master

# Grab turtlebot3_world model assets (so model://turtlebot3_world resolves)
# We only need models/ folder to satisfy Gazebo Sim resource search.
RUN git clone --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git /tmp/tb3_sim && \
    mkdir -p /opt/gz_models && \
    cp -r /tmp/tb3_sim/turtlebot3_gazebo/models/* /opt/gz_models/ && \
    rm -rf /tmp/tb3_sim

# Important: Gazebo Sim searches model:// via GZ_SIM_RESOURCE_PATH
ENV GZ_SIM_RESOURCE_PATH=/opt/gz_models

# rosdep: repo's package.xml has some keys that may not resolve cleanly on Jazzy;
# keep build deterministic by skipping problematic keys and installing needed deps via apt above.
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys="gazebo_ros_pkgs ament_python turtlebot3_gazebo turtlebot3_navigation2 navigation2 nav2_common slam_toolbox"

# Build
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install

# --- noVNC startup script ---
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 6080 5900
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "-lc", "source /opt/ros2_ws/install/setup.bash && exec bash"]

