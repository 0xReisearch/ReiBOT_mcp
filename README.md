# ReiBOT - ROS MCP Robot Control System - v0.01

First implementation for a 6-DOF robot arm control system that integrates ROS 2, Model Context Protocol (MCP), and web visualization for both real and simulated robot control through Claude AI.

Tested only in simulation mode.

## Features

- **6-DOF Robot Arm Control**: Full 6 degrees of freedom with realistic joint limits and constraints
- **MCP Integration**: Control robot arm through Claude AI using Model Context Protocol
- **ROS 2 Support**: Full integration with ROS 2 for real robot control
- **Web Visualization**: Real-time 3D visualization of robot arm with smooth animations
- **Dual Mode**: Supports both simulation and real robot operation
- **Real-time Updates**: Live state synchronization between all components
- **Safety Features**: Joint limits, movement constraints, and emergency stop functionality

## System Architecture

The system consists of three main components:

1. **ROS Bridge Server** (Port 9090): Handles ROS communication
2. **MCP Server** (Port 8000/8001): Provides MCP tools for Claude and HTTP endpoints for web interface
3. **Web Visualization** (Port 5000): Real-time 3D robot arm visualization with smooth animations

## Prerequisites

- Ubuntu 22.04 or later
- ROS 2 Kilted
- Python 3.10+
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
git clone https://github.com/0xReisearch/ReiBOT_mcp
cd ReiBOT_mcp
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
pip install rclpy

# Install visualization dependencies
pip install Pillow tornado pymongo

# Install additional dependencies
pip install argcomplete PyYAML
```

### 6. Make Scripts Executable

```bash
chmod +x start_mcp_server.sh
```

## Usage

### Starting the System

You need to start three components in separate terminals:

#### Terminal 1: ROS Bridge Server
```bash
cd /path/to/ReiBOT_mcp
source /opt/ros/kilted/setup.bash
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

#### Terminal 2: MCP Server
```bash
cd /path/to/ReiBOT_mcp
./start_mcp_server.sh
```

#### Terminal 3: Web Visualization
```bash
cd /path/to/ReiBOT_mcp
source venv/bin/activate
python web_visualization.py --host 0.0.0.0 --port 5000
```

### Accessing the Web Interface

Open your browser and navigate to:
```
http://localhost:5000
```

You'll see a real-time 3D visualization of the 6-DOF robot arm that updates with smooth animations when Claude sends movement commands.

## 6-DOF Robot Arm Configuration

| Joint | Name | Range | Function |
|-------|------|-------|----------|
| 0 | Base | ±180° | Horizontal rotation |
| 1 | Shoulder | ±90° | Vertical arm movement |
| 2 | Elbow | ±135° | Forearm articulation |
| 3 | Wrist Roll | ±180° | End effector rotation |
| 4 | Wrist Pitch | ±90° | End effector tilt |
| 5 | Wrist Yaw | ±180° | End effector pan |

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
    "reibot-arm": {
      "command": "bash",
      "args": ["-c", "cd /path/to/your/ReiBOT_mcp && ./start_mcp_server.sh"],
      "env": {
        "PATH": "/usr/bin:/bin:/opt/ros/kilted/bin"
      }
    }
  }
}
```

**Important:** Replace `/path/to/your/ReiBOT_mcp` with the actual path to your project directory.

### 4. Restart Claude Desktop

Close and restart Claude Desktop for the configuration to take effect.

### 5. Verify Integration

In Claude, you should now be able to use commands like:

- "Move the robot arm's shoulder joint down by 20 degrees"
- "Set the robot arm to a reaching position"
- "Make the robot wave at someone"
- "Get the current position of all joints on the robot arm"
- "Reset the robot arm to home position"

## Available MCP Tools

Claude has access to these robot arm control tools:

### Joint Control
- **`move_joint`**: Move a specific joint by angle (0=base, 1=shoulder, 2=elbow, 3=wrist_roll, 4=wrist_pitch, 5=wrist_yaw)
- **`move_joints`**: Move multiple joints simultaneously with 6 angles
- **`set_joint_angles`**: Set absolute joint positions

### End Effector Control
- **`move_end_effector`**: Move end effector in Cartesian space (x, y, z)
- **`rotate_wrist`**: Rotate wrist joints (roll, pitch, yaw)

### Predefined Gestures
- **`perform_gesture`**: Execute predefined movements (wave, reach_up, reach_forward, point_left, point_right)

### System Control
- **`get_status`**: Get current robot arm status and joint angles
- **`stop_robot`**: Emergency stop all movement
- **`reset_to_home`**: Return to home position (all joints at 0°)
- **`set_mode`**: Set operation mode (real or simulation)

## Safety Features

- **Joint limits**: Each joint has realistic angular constraints
- **Movement limits**: Commands are clamped to safe ranges (±30° per command)
- **Emergency stop**: Immediate halt of all movement
- **Home position**: Safe default position for initialization
- **Status monitoring**: Real-time feedback on robot state

## Troubleshooting

### Common Issues

#### 1. BSON/PyMongo Conflicts
```bash
# Remove conflicting bson package
pip uninstall bson -y
# Install correct pymongo
pip install pymongo
```

#### 2. Missing Dependencies
```bash
pip install argcomplete PyYAML Pillow tornado
```

#### 3. ROS Bridge Won't Start
```bash
# Make sure you source ROS environment
source /opt/ros/kilted/setup.bash
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

#### 4. MCP Server Connection Issues
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
ReiBOT_mcp/
├── integrated_mcp_server.py    # Main MCP server with 6-DOF robot arm control
├── ros_interface.py            # ROS 2 integration and joint control
├── web_visualization.py        # Web visualization server
├── start_mcp_server.sh        # MCP server startup script
├── requirements.txt           # Python dependencies
├── templates/                 # HTML templates
│   └── index.html            # Web interface template
├── static/                    # Static web assets
│   ├── css/
│   │   └── styles.css        # Web interface styling
│   └── js/
│       └── visualization.js   # 3D robot arm rendering with Three.js
├── venv/                      # Python virtual environment
└── README.md                  # This file
```

### Extending the System

- Add new MCP tools in `integrated_mcp_server.py`
- Modify robot arm behavior in `ros_interface.py`
- Enhance 3D visualization in `static/js/visualization.js`
- Update web interface in `templates/index.html`

## Web Interface Features

- **3D Robot Arm**: Real-time visualization with color-coded joints
- **Smooth Animations**: Joints move smoothly to target positions
- **Joint Angle Display**: Live monitoring of all 6 joint positions
- **End Effector Position**: Current Cartesian coordinates
- **Command History**: Log of all commands from Claude with timestamps
- **System Status**: Connection status, battery level, and operation mode

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Verify all dependencies are installed correctly
3. Ensure all three components are running
4. Check that Claude Desktop configuration is correct

## References

- ROS 2 Documentation: https://docs.ros.org/en/kilted/
- FastMCP: https://github.com/jlowin/fastmcp
- Three.js: https://threejs.org/

---

Made with ❤️ by [Rei Network](https://reisearch.box)

