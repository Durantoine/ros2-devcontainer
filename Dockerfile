xFROM osrf/ros:humble-desktop

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces

# ROS2 configuration
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

CMD ["/bin/bash"]