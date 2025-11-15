#!/bin/bash

echo "🚀 Starting ROS2 Development Environment..."

# Start container in background
docker-compose up -d

# Wait for services to be ready
echo "⏳ Waiting for services to start..."
sleep 5

# Check if services are running
echo ""
echo "Checking services status..."

if docker exec ros2_dev pgrep x11vnc > /dev/null; then
    echo "✅ VNC server is running"
else
    echo "⚠️  VNC server may not be ready yet"
fi

if docker exec ros2_dev pgrep -f novnc_proxy > /dev/null; then
    echo "✅ noVNC web interface is running"
else
    echo "⚠️  noVNC may not be ready yet"
fi

echo ""
echo "======================================"
echo "🎯 Access Methods:"
echo "======================================"
echo ""
echo "1️⃣  Web Browser (Recommended):"
echo "   🌐 http://localhost:6080"
echo "   Click 'Connect' to see GUI applications"
echo ""
echo "2️⃣  Native VNC Client:"
echo "   📺 vnc://localhost:5900"
echo "   Or run: open vnc://localhost:5900"
echo ""
echo "3️⃣  VS Code Dev Containers:"
echo "   1. Install 'Dev Containers' extension"
echo "   2. Click the blue icon in bottom-left"
echo "   3. Select 'Reopen in Container'"
echo ""
echo "======================================"
echo ""

# Enter the container
docker-compose exec ros2 bash