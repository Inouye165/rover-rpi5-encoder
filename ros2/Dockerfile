FROM ros:jazzy-ros-base-noble

# Set non-interactive debian frontend
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies and ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-ros \
    ros-jazzy-diagnostic-updater \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-requests \
    python3-pytest \
    curl \
    jq \
    iproute2 \
    procps \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Delete existing ubuntu user if it has UID 1000 to prevent conflict,
# and create our own non-root rover user with UID/GID 1000 matching the host 'ron' user
RUN id -u ubuntu &>/dev/null && userdel -r ubuntu || true \
    && groupadd --gid 1000 rover \
    && useradd --uid 1000 --gid 1000 --shell /bin/bash --create-home rover \
    && echo "rover ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/rover \
    && chmod 0440 /etc/sudoers.d/rover

# Set up the workspace directory
WORKDIR /ros2_ws
RUN chown -R rover:rover /ros2_ws

# Switch to the non-root rover user
USER rover

# Initialize rosdep
RUN rosdep update --rosdistro jazzy

# Configure shell environment for rover user
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/rover/.bashrc \
    && echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /home/rover/.bashrc

# Expose build environment variables
ENV ROS_DISTRO=jazzy

# Set default entrypoint to source setup.bash before running commands
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
