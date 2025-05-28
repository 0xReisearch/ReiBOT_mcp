#!/bin/bash

# MCP Robot Server Startup Script
# This script properly sets up ROS 2 environment and starts the MCP server

set -e

echo "Starting MCP Robot Server..." >&2

# Source ROS 2 environment
echo "Setting up ROS 2 environment..." >&2
source /opt/ros/kilted/setup.bash

# Add ROS Python path to PYTHONPATH so our venv can find ROS modules
export PYTHONPATH="/opt/ros/kilted/lib/python3.12/site-packages:$PYTHONPATH"

# Activate virtual environment
echo "Activating virtual environment..." >&2
source venv/bin/activate

# Start the MCP server
echo "Starting MCP server..." >&2
python integrated_mcp_server.py --mode simulation --host 0.0.0.0 --port 8000 --ros-bridge ws://localhost:9090 