FROM osrf/ros:humble-desktop

# Install system dependencies and noVNC
RUN apt-get update && apt-get install -y \
    wget \
    python3-pip \
    x11vnc \
    xvfb \
    supervisor \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Install noVNC
RUN mkdir -p /opt/noVNC/utils/websockify && \
    wget -qO- https://github.com/novnc/noVNC/archive/v1.4.0.tar.gz | tar xz --strip 1 -C /opt/noVNC && \
    wget -qO- https://github.com/novnc/websockify/archive/v0.11.0.tar.gz | tar xz --strip 1 -C /opt/noVNC/utils/websockify && \
    ln -s /opt/noVNC/vnc.html /opt/noVNC/index.html

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-foxglove-bridge \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces

# Copy workspace helpers
COPY workspace_helpers.sh /workspace_helpers.sh

# Configure supervisor for noVNC and Xvfb
RUN mkdir -p /var/log/supervisor
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# ROS2 configuration
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc && \
    echo "export DISPLAY=:1" >> ~/.bashrc && \
    echo "source /workspace_helpers.sh" >> ~/.bashrc

# Expose noVNC port
EXPOSE 6080

# Start supervisor (manages Xvfb and noVNC)
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]