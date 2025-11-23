#!/bin/bash
# Auto-start VNC/noVNC services if not already running

if ! pgrep -x supervisord > /dev/null; then
    echo "🚀 Starting VNC/noVNC services..."
    /usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf
    sleep 2
    echo "✅ VNC started at http://localhost:6080"
else
    echo "✅ VNC services already running"
fi
