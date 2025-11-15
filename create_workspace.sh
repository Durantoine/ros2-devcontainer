#!/bin/bash
# Script pour créer un nouveau workspace ROS2

if [ -z "$1" ]; then
    echo "Usage: ./create_workspace.sh <workspace_name>"
    echo "Example: ./create_workspace.sh my_robot_project"
    exit 1
fi

WORKSPACE_NAME=$1
WORKSPACE_PATH="./workspaces/$WORKSPACE_NAME"

if [ -d "$WORKSPACE_PATH" ]; then
    echo "❌ Workspace '$WORKSPACE_NAME' already exists!"
    exit 1
fi

echo "Creating new ROS2 workspace: $WORKSPACE_NAME"
mkdir -p "$WORKSPACE_PATH"

echo "✅ Workspace created at: $WORKSPACE_PATH"
echo ""
echo "Next steps:"
echo "1. Open VSCode in /Users/durantoine/Dev/ros2_devcontainer"
echo "2. Reopen in Container (Dev Containers: Reopen in Container)"
echo "3. Your workspace will be available at /workspaces/$WORKSPACE_NAME"
echo "4. The VNC interface will open automatically at http://localhost:6080"
