#!/usr/bin/env python3
"""
ROS Integration Module for MCP Robot Server
Uses actual ROS 2 Python API (rclpy) for real ROS integration
"""

import asyncio
import threading
import time
import sys
from enum import Enum
from typing import Dict, Any, Optional, Callable

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class OperationMode(str, Enum):
    REAL = "real"
    SIMULATION = "simulation"


class ROSNode(Node):
    """ROS 2 Node for robot control"""
    
    def __init__(self, mode: OperationMode, on_state_update: Optional[Callable[[Dict[str, Any]], None]] = None):
        super().__init__('mcp_robot_controller')
        self.mode = mode
        self.on_state_update = on_state_update
        
        # Publishers
        topic = '/cmd_vel'  # Use /cmd_vel for both real and simulation
        self.cmd_vel_publisher = self.create_publisher(Twist, topic, 10)
        
        # Subscribers - listen to commands from web interface
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Robot state
        self.robot_state = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "status": "idle",
            "battery": 100.0,
            "mode": mode.value
        }
        
        # Timer for state updates
        self.create_timer(1.0, self.update_state)
        
        self.get_logger().info(f'ROS node initialized in {mode.value} mode')
        
    def cmd_vel_callback(self, msg):
        """Handle incoming cmd_vel messages from web interface"""
        self.get_logger().info(f'Received cmd_vel: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')
        
        # Update robot state based on received command
        if msg.linear.x != 0.0 or msg.linear.y != 0.0 or msg.linear.z != 0.0:
            self.robot_state["status"] = "moving"
            # Simulate movement
            self.robot_state["position"]["x"] += msg.linear.x * 0.1
            self.robot_state["position"]["y"] += msg.linear.y * 0.1
            self.robot_state["position"]["z"] += msg.linear.z * 0.1
        elif msg.angular.x != 0.0 or msg.angular.y != 0.0 or msg.angular.z != 0.0:
            self.robot_state["status"] = "rotating"
            # Simulate rotation
            self.robot_state["orientation"]["roll"] += msg.angular.x * 0.1
            self.robot_state["orientation"]["pitch"] += msg.angular.y * 0.1
            self.robot_state["orientation"]["yaw"] += msg.angular.z * 0.1
        else:
            self.robot_state["status"] = "idle"
        
        # Notify state update
        if self.on_state_update:
            self.on_state_update(self.robot_state)
        
    def publish_twist(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                     angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Publish twist message"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.linear.z = float(linear_z)
        msg.angular.x = float(angular_x)
        msg.angular.y = float(angular_y)
        msg.angular.z = float(angular_z)
        
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Published twist: linear=({linear_x}, {linear_y}, {linear_z}), angular=({angular_x}, {angular_y}, {angular_z})')
        
    def update_state(self):
        """Update robot state periodically"""
        if self.on_state_update:
            self.on_state_update(self.robot_state)


class ROSInterface:
    """
    Interface for ROS communication using actual ROS 2 Python API
    Handles both real and simulation scenarios
    """
    
    def __init__(self, mode: OperationMode = OperationMode.SIMULATION,
                 ros_bridge_url: str = "ws://localhost:9090",
                 on_state_update: Optional[Callable[[Dict[str, Any]], None]] = None):
        self.mode = mode
        self.ros_bridge_url = ros_bridge_url
        self.on_state_update = on_state_update
        self.connected = False
        self.node = None
        self.executor = None
        self.ros_thread = None
        
        # Initialize robot state
        self.robot_state = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "status": "idle",
            "battery": 100.0,
            "mode": mode.value
        }
        
    async def start(self):
        """Start ROS client connection"""
        try:
            # Initialize ROS 2
            if not rclpy.ok():
                rclpy.init()
            
            # Create ROS node
            self.node = ROSNode(self.mode, self._on_ros_state_update)
            
            # Create executor and run in separate thread
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            
            # Start ROS spinning in a separate thread
            self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
            self.ros_thread.start()
            
            self.connected = True
            print(f"ROS 2 node started successfully in {self.mode.value} mode", file=sys.stderr)
            
        except Exception as e:
            print(f"ROS connection failed: {e}", file=sys.stderr)
            self._setup_simulation_fallback()
        
    def _spin_ros(self):
        """Spin ROS executor in separate thread"""
        try:
            self.executor.spin()
        except Exception as e:
            print(f"ROS spinning error: {e}", file=sys.stderr)
            self.connected = False
            
    def _on_ros_state_update(self, state):
        """Handle state updates from ROS node"""
        self.robot_state.update(state)
        if self.on_state_update:
            self.on_state_update(self.robot_state)
        
    def _setup_simulation_fallback(self):
        """Set up simulation fallback when ROS connection fails"""
        print("Running in offline simulation mode", file=sys.stderr)
        self.connected = False
        self.robot_state["status"] = "offline_simulation"
        
        # Notify state update
        if self.on_state_update:
            self.on_state_update(self.robot_state)
        
    def move_robot(self, x: float, y: float, z: float = 0.0):
        """Send movement command to robot"""
        if self.connected and self.node:
            # Use ROS to publish movement
            self.node.publish_twist(linear_x=x, linear_y=y, linear_z=z)
            self.robot_state["status"] = "moving"
            return True
        else:
            # Offline simulation mode - simulate movement locally
            print(f"Simulating movement: x={x}, y={y}, z={z}", file=sys.stderr)
            self.robot_state["position"]["x"] += x * 0.1  # Simulate movement
            self.robot_state["position"]["y"] += y * 0.1
            self.robot_state["position"]["z"] += z * 0.1
            self.robot_state["status"] = "moving"
            
            # Notify state update
            if self.on_state_update:
                self.on_state_update(self.robot_state)
            return True
        
    def rotate_robot(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
        """Send rotation command to robot"""
        if self.connected and self.node:
            # Use ROS to publish rotation
            self.node.publish_twist(angular_x=roll, angular_y=pitch, angular_z=yaw)
            self.robot_state["status"] = "rotating"
            return True
        else:
            # Offline simulation mode - simulate rotation locally
            print(f"Simulating rotation: roll={roll}, pitch={pitch}, yaw={yaw}", file=sys.stderr)
            self.robot_state["orientation"]["roll"] += roll * 0.1  # Simulate rotation
            self.robot_state["orientation"]["pitch"] += pitch * 0.1
            self.robot_state["orientation"]["yaw"] += yaw * 0.1
            self.robot_state["status"] = "rotating"
            
            # Notify state update
            if self.on_state_update:
                self.on_state_update(self.robot_state)
            return True
        
    def stop_robot(self):
        """Stop robot movement"""
        if self.connected and self.node:
            # Use ROS to publish stop command
            self.node.publish_twist()  # All zeros
            self.robot_state["status"] = "idle"
            return True
        else:
            # Offline simulation mode - simulate stop locally
            print("Simulating robot stop", file=sys.stderr)
            self.robot_state["status"] = "idle"
            
            # Notify state update
            if self.on_state_update:
                self.on_state_update(self.robot_state)
            return True
        
    def set_mode(self, mode: OperationMode):
        """Change operation mode"""
        if self.mode == mode:
            return True
            
        # Update mode
        self.mode = mode
        self.robot_state["mode"] = mode.value
        
        # Notify state update
        if self.on_state_update:
            self.on_state_update(self.robot_state)
            
        return True
        
    def get_state(self):
        """Get current robot state"""
        return self.robot_state
        
    def shutdown(self):
        """Shutdown ROS client"""
        self.connected = False
        
        if self.executor:
            self.executor.shutdown()
            
        if self.node:
            self.node.destroy_node()
            
        if rclpy.ok():
            rclpy.shutdown()
