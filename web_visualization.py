#!/usr/bin/env python3
"""
Web Visualization Server for Robot Simulation
Provides a web interface to visualize robot state and control in simulation
"""

import os
import json
import asyncio
import threading
import time
import requests
from flask import Flask, render_template, send_from_directory
from flask_socketio import SocketIO, emit
import sys

# Initialize Flask app
app = Flask(__name__, 
            static_folder='static',
            template_folder='templates')
app.config['SECRET_KEY'] = 'robotvisualization'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
robot_state = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "status": "idle",
    "battery": 100.0,
    "mode": "simulation"
}

# MCP server connection
mcp_server_url = "http://localhost:8001"

def poll_mcp_server():
    """Poll MCP server for robot state updates"""
    global robot_state
    
    while True:
        try:
            # Try to get status from MCP server
            response = requests.get(f"{mcp_server_url}/status", timeout=1)
            if response.status_code == 200:
                new_state = response.json()
                if new_state != robot_state:
                    robot_state = new_state
                    print(f"Robot state updated: {robot_state}", file=sys.stderr)
                    # Broadcast to all connected clients
                    socketio.emit('robot_state_update', robot_state)
        except Exception as e:
            # MCP server not available, use simulation
            pass
        
        time.sleep(0.1)  # Poll every 100ms

# Start polling thread
polling_thread = threading.Thread(target=poll_mcp_server, daemon=True)
polling_thread.start()

# Flask routes
@app.route('/')
def index():
    """Render main visualization page"""
    return render_template('index.html')

@app.route('/static/<path:path>')
def serve_static(path):
    """Serve static files"""
    return send_from_directory('static', path)

# SocketIO events
@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print('Client connected', file=sys.stderr)
    # Send current robot state to new client
    emit('robot_state_update', robot_state)

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print('Client disconnected', file=sys.stderr)

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description="Web Visualization Server for Robot Simulation")
    parser.add_argument(
        "--host", 
        type=str, 
        default="0.0.0.0",
        help="Host address to bind the server to"
    )
    parser.add_argument(
        "--port", 
        type=int, 
        default=5000,
        help="Port to run the server on"
    )
    parser.add_argument(
        "--mcp-server", 
        type=str, 
        default="http://localhost:8001",
        help="MCP server URL"
    )
    
    args = parser.parse_args()
    mcp_server_url = args.mcp_server
    
    print(f"Starting web visualization server on {args.host}:{args.port}", file=sys.stderr)
    print(f"Connecting to MCP server at {mcp_server_url}", file=sys.stderr)
    
    # Run Flask app
    socketio.run(app, host=args.host, port=args.port, debug=False, allow_unsafe_werkzeug=True)
