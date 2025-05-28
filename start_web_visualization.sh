#!/bin/bash

# Web Visualization Startup Script
# This script starts the web visualization server

set -e

echo "Starting Web Visualization Server..."

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Install required packages if not already installed
pip install flask flask-socketio roslibpy

# Start the web visualization server
echo "Starting web visualization server..."
python web_visualization.py --host 0.0.0.0 --port 5000 --ros-bridge ws://localhost:9090 