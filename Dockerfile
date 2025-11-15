FROM --platform=linux/arm64 osrf/ros:jazzy-desktop

# Install VNC, noVNC, window manager, and development tools
RUN apt-get update && apt-get install -y \
    vim \
    nano \
    git \
    wget \
    curl \
    x11vnc \
    xvfb \
    fluxbox \
    supervisor \
    net-tools \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

# Install pip and Python development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Install uv for fast Python package management (optional)
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# Install ROS2 Python dependencies (critical for ROS2 to work)
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-catkin-pkg \
    python3-catkin-pkg-modules \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python development tools using pip
RUN pip3 install --break-system-packages \
    pytest \
    pytest-cov \
    flake8 \
    black \
    mypy

# Set up workspaces directory
RUN mkdir -p /workspaces
WORKDIR /workspaces

# Create supervisord config for auto-starting services
RUN mkdir -p /var/log/supervisor
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Configure noVNC with autoconnect
RUN ln -s /usr/share/novnc/vnc.html /usr/share/novnc/index.html && \
    echo '<!DOCTYPE html><html><head><meta http-equiv="refresh" content="0; url=vnc.html?autoconnect=true&resize=scale"></head><body>Redirecting...</body></html>' > /usr/share/novnc/auto.html

# Configure Fluxbox menu
RUN mkdir -p ~/.fluxbox && \
    echo '[begin] (Fluxbox)' > ~/.fluxbox/menu && \
    echo '  [restart] (Restart)' >> ~/.fluxbox/menu && \
    echo '  [exit] (Exit)' >> ~/.fluxbox/menu && \
    echo '[end]' >> ~/.fluxbox/menu

# Source ROS2 automatically and ensure no venv is active
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "# Ensure no venv is active (ROS2 needs system Python packages)" >> ~/.bashrc
RUN echo "unset VIRTUAL_ENV" >> ~/.bashrc

# Add helpful aliases for building in current directory
RUN echo "alias build='colcon build --symlink-install'" >> ~/.bashrc
RUN echo "alias cbuild='colcon build --symlink-install --cmake-clean-cache'" >> ~/.bashrc
RUN echo "alias source_ws='if [ -f ./install/setup.bash ]; then source ./install/setup.bash; else echo \"No install/setup.bash in current directory\"; fi'" >> ~/.bashrc

# Load workspace helpers if available
RUN echo "if [ -f /workspace_helpers.sh ]; then source /workspace_helpers.sh; fi" >> ~/.bashrc

# Expose VNC and noVNC ports
EXPOSE 5900 6080

# Create entrypoint script that starts supervisord for GUI services
RUN echo '#!/bin/bash\n\
set -e\n\
echo "🚀 Starting VNC/noVNC services via supervisord..."\n\
/usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf &\n\
sleep 3\n\
echo "✅ VNC available at: localhost:5900"\n\
echo "✅ noVNC web interface at: http://localhost:6080"\n\
exec /bin/bash -l\n\
' > /entrypoint.sh && chmod +x /entrypoint.sh

CMD ["/entrypoint.sh"]