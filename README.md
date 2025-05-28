# ReuBOT - ROS MCP Robot Control System - v0.01

First implementation for a robot control system that integrates ROS 2, Model Context Protocol (MCP), and web visualization for both real and simulated robot control through Claude AI.

Tested only in simulation mode.

## Features

- **MCP Integration**: Control robots through Claude AI using Model Context Protocol
- **ROS 2 Support**: Full integration with ROS 2 for real robot control
- **Web Visualization**: Real-time 3D visualization of robot state and movements
- **Dual Mode**: Supports both simulation and real robot operation
- **Real-time Updates**: Live state synchronization between all components

## System Architecture

The system consists of three main components:

1. **ROS Bridge Server** (Port 9090): Handles ROS communication
2. **MCP Server** (Port 8000/8001): Provides MCP tools for Claude and HTTP endpoints for web interface
3. **Web Visualization** (Port 5000): Real-time 3D robot visualization

## Prerequisites

- Ubuntu 22.04 or later
- ROS 2 Kilted
- Python 3.12+
- node.js (for some dependencies)

## Installation

### 1. Install ROS 2

```bash
# Install ROS 2 (if not already installed)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-kilted-desktop  # Replace 'kilted' with your ROS 2 version
```

### 2. Install ROS Bridge Server

```bash
sudo apt-get install ros-kilted-rosbridge-server  # Replace 'kilted' with your ROS 2 version
```

### 3. Clone and Setup Project

```bash
git clone <your-repo-url>
cd ros
```

### 4. Create Python Virtual Environment

```bash
python3 -m venv venv
source venv/bin/activate
```

### 5. Install Python Dependencies

```bash
# Install core dependencies
pip install fastmcp fastapi uvicorn flask flask-socketio requests

# Install ROS dependencies
pip install rclpy geometry-msgs

# Install visualization dependencies
pip install Pillow tornado pymongo

# Install additional dependencies
pip install argcomplete
```

### 6. Make Scripts Executable

```bash
chmod +x start_mcp_server.sh
chmod +x start_web_visualization.sh
```

## Usage

### Starting the System

You need to start three components in separate terminals:

#### Terminal 1: ROS Bridge Server
```bash
cd /path/to/ros
source venv/bin/activate
source /opt/ros/kilted/setup.bash
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

#### Terminal 2: MCP Server
```bash
cd /path/to/ros
source venv/bin/activate
./start_mcp_server.sh
```

#### Terminal 3: Web Visualization
```bash
cd /path/to/ros
source venv/bin/activate
python web_visualization.py --host 0.0.0.0 --port 5000
```

### Accessing the Web Interface

Open your browser and navigate to:
```
http://localhost:5000
```

You'll see a real-time 3D visualization of the robot that updates when the agent sends movement commands.

## Rei Network Integration

Work in progress!

## Claude Integration

### 1. Install Claude Desktop

Download and install Claude Desktop from Anthropic.

### 2. Configure MCP in Claude

Edit your Claude Desktop configuration file:

**On macOS:**
```bash
~/Library/Application Support/Claude/claude_desktop_config.json
```

**On Linux:**
```bash
~/.config/Claude/claude_desktop_config.json
```

**On Windows:**
```bash
%APPDATA%\Claude\claude_desktop_config.json
```

### 3. Add MCP Server Configuration

Add this configuration to your `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "ros-robot-control": {
      "command": "bash",
      "args": ["-c", "cd /path/to/your/ros && ./start_mcp_server.sh"],
      "env": {
        "PATH": "/path/to/your/ros/venv/bin:/usr/bin:/bin"
      }
    }
  }
}
```

**Important:** Replace `/path/to/your/ros` with the actual path to your project directory.

### 4. Restart Claude Desktop

Close and restart Claude Desktop for the configuration to take effect.

### 5. Verify Integration

In Claude, you should now be able to use commands like:

- "Move the robot forward by 1 meter"
- "Rotate the robot 90 degrees to the left"
- "Get the current robot status"
- "Stop the robot"

## Available MCP Tools

Claude has access to these robot control tools:

- **`get_status`**: Get current robot status and state information
- **`move_robot`**: Move robot to specified position coordinates (x, y, z)
- **`rotate_robot`**: Rotate robot to specified orientation (roll, pitch, yaw)
- **`stop_robot`**: Stop all robot movement immediately
- **`set_mode`**: Set operation mode (real or simulation)

## Troubleshooting

### Common Issues

#### 1. BSON/PyMongo Conflicts
```bash
# Remove conflicting bson package
pip uninstall bson -y
# Install correct pymongo
pip install pymongo
```

#### 2. Missing argcomplete
```bash
pip install argcomplete
```

#### 3. Tornado Missing
```bash
pip install tornado
```

#### 4. ROS Bridge Won't Start
```bash
# Make sure you're not in the virtual environment for ROS commands
deactivate
source /opt/ros/kilted/setup.bash
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

#### 5. MCP Server Connection Issues
- Ensure all three components are running
- Check that ports 8000, 8001, 9090, and 5000 are not blocked
- Verify the path in Claude's configuration is correct

### Port Usage

- **9090**: ROS Bridge WebSocket server
- **8000**: MCP server (for Claude communication)
- **8001**: HTTP endpoints (for web interface polling)
- **5000**: Web visualization server

### Logs and Debugging

- MCP server logs go to stderr
- Web visualization logs go to stderr
- ROS bridge logs appear in the terminal
- Check browser console for web interface issues

## Development

### Project Structure

```
ros/
├── integrated_mcp_server.py    # Main MCP server with ROS integration
├── ros_interface.py           # ROS communication layer
├── web_visualization.py       # Web visualization server
├── start_mcp_server.sh       # MCP server startup script
├── start_web_visualization.sh # Web server startup script
├── templates/                # HTML templates
│   └── index.html
├── static/                   # Static web assets
│   ├── style.css
│   └── script.js
├── venv/                     # Python virtual environment
└── README.md                 # This file
```

### Extending the System

- Add new MCP tools in `integrated_mcp_server.py`
- Modify robot behavior in `ros_interface.py`
- Enhance visualization in `templates/index.html` and `static/`

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Verify all dependencies are installed correctly
3. Ensure all three components are running
4. Check that Claude Desktop configuration is correct

## References

ROS homepage: https://www.ros.org/

---

Made with ❤️ by [Rei Network](https://reisearch.box)

