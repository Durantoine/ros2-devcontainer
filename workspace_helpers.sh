#!/bin/bash
# Helper functions pour gérer les workspaces dans le container
# À sourcer dans le container: source /workspaces/../workspace_helpers.sh

# Fonction pour builder un workspace spécifique
build_workspace() {
    if [ -z "$1" ]; then
        echo "Usage: build_workspace <workspace_name>"
        echo "Available workspaces:"
        ls /workspaces/
        return 1
    fi

    # Remove trailing slash if present
    local WORKSPACE="${1%/}"
    local WORKSPACE_PATH="/workspaces/$WORKSPACE"

    if [ ! -d "$WORKSPACE_PATH" ]; then
        echo "❌ Workspace '$WORKSPACE' not found!"
        return 1
    fi

    echo "🔨 Building workspace: $WORKSPACE"
    cd "$WORKSPACE_PATH"

    # Build directly in the workspace directory
    # colcon automatically uses build/, install/, and log/ in the current directory
    colcon build --symlink-install

    if [ $? -eq 0 ]; then
        echo "✅ Build complete!"
        echo "📦 To use this workspace, run: source $WORKSPACE_PATH/install/setup.bash"
    else
        echo "❌ Build failed!"
        return 1
    fi
}

# Fonction pour switcher entre workspaces
switch_workspace() {
    if [ -z "$1" ]; then
        echo "Usage: switch_workspace <workspace_name>"
        echo "Available workspaces:"
        ls /workspaces/
        return 1
    fi

    # Remove trailing slash if present
    local WORKSPACE="${1%/}"
    local WORKSPACE_PATH="/workspaces/$WORKSPACE"

    if [ ! -d "$WORKSPACE_PATH" ]; then
        echo "❌ Workspace '$WORKSPACE' not found!"
        return 1
    fi

    cd "$WORKSPACE_PATH"

    echo "✅ Switched to workspace: $WORKSPACE"
    echo "📁 Current directory: $WORKSPACE_PATH"
    echo ""

    if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
        source "$WORKSPACE_PATH/install/setup.bash"
        echo "✅ Workspace environment sourced"
    else
        echo "⚠️  Workspace not built yet. Run 'build_workspace $WORKSPACE' first"
    fi
}

# Fonction pour lister tous les workspaces
list_workspaces() {
    echo "📦 Available ROS2 Workspaces:"
    echo ""
    for ws in /workspaces/*/; do
        if [ -d "$ws" ]; then
            local ws_name=$(basename "$ws")
            # Count package.xml files in src/ subdirectories (ROS2 convention)
            local pkg_count=$(find "$ws/src" -name "package.xml" 2>/dev/null | wc -l | tr -d ' ')

            # Check if workspace is built
            if [ -d "$ws/install" ]; then
                echo "  • $ws_name ($pkg_count packages) ✅ built"
            else
                echo "  • $ws_name ($pkg_count packages) ⚠️  not built"
            fi
        fi
    done
    echo ""

    # Show current directory if in a workspace
    if [[ "$PWD" == /workspaces/* ]]; then
        local current=$(basename "$PWD")
        echo "Current workspace: $current"
    else
        echo "Not currently in a workspace directory"
    fi
}

# Aliases pratiques
alias lsws='list_workspaces'
alias ws='switch_workspace'
alias bws='build_workspace'

echo "✅ Workspace helpers loaded!"
echo "Commands available:"
echo "  - list_workspaces (or lsws) : List all workspaces"
echo "  - switch_workspace <name> (or ws) : Switch to a workspace"
echo "  - build_workspace <name> (or bws) : Build a workspace"
