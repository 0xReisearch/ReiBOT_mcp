#!/usr/bin/env python3
"""
ROS Interface for 6-DOF Robot Arm Control
Supports both real and simulation scenarios with proper joint control
"""

import asyncio
import threading
import time
import sys
from enum import Enum
from typing import Dict, Any, Optional, Callable
import math

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class OperationMode(str, Enum):
    REAL = "real"
    SIMULATION = "simulation"

class ROSNode(Node):
    """ROS 2 Node for 6-DOF Robot Arm Control"""
    
    def __init__(self, on_state_update: Optional[Callable] = None):
        super().__init__('reibot_arm_controller')
        
        self.on_state_update = on_state_update
        
        # Robot arm state (6-DOF)
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # degrees
        self.end_effector_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.status = "idle"
        self.battery = 100.0
        
        # Joint limits (degrees) - realistic robot arm limits
        self.joint_limits = [
            (-180, 180),  # Base rotation
            (-90, 90),    # Shoulder
            (-135, 135),  # Elbow
            (-180, 180),  # Wrist roll
            (-90, 90),    # Wrist pitch
            (-180, 180)   # Wrist yaw
        ]
        
        # Movement scaling factors
        self.joint_step_size = 15.0  # degrees per command
        self.position_step_size = 0.5  # meters per command
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.joint_cmd_sub = self.create_subscription(
            JointState, '/joint_commands', self.joint_command_callback, 10
        )
        
        # Timer for state updates
        self.state_timer = self.create_timer(0.1, self.publish_state)  # 10Hz
        
        self.get_logger().info("6-DOF Robot Arm Controller initialized")
    
    def clamp_joint_angle(self, joint_index: int, angle: float) -> float:
        """Clamp joint angle to valid range"""
        min_angle, max_angle = self.joint_limits[joint_index]
        return max(min_angle, min(max_angle, angle))
    
    def forward_kinematics(self) -> Dict[str, float]:
        """Calculate end effector position from joint angles (simplified)"""
        # Simplified forward kinematics for visualization
        # In a real robot, this would be much more complex
        
        j1, j2, j3, j4, j5, j6 = [math.radians(a) for a in self.joint_angles]
        
        # Link lengths (from armConfig in visualization)
        l1 = 1.5  # joint1 height
        l2 = 1.2  # joint2 height  
        l3 = 0.8  # joint3 height
        l4 = 0.6  # joint4 height
        l5 = 0.4  # joint5 height
        l6 = 0.3  # end effector height
        
        # Simplified calculation
        x = (l2 * math.cos(j2) + l3 * math.cos(j2 + j3) + 
             l4 * math.cos(j2 + j3) + l5 * math.cos(j2 + j3) + l6) * math.cos(j1)
        y = (l2 * math.cos(j2) + l3 * math.cos(j2 + j3) + 
             l4 * math.cos(j2 + j3) + l5 * math.cos(j2 + j3) + l6) * math.sin(j1)
        z = 0.5 + l1 + l2 * math.sin(j2) + l3 * math.sin(j2 + j3)  # base height + calculations
        
        return {"x": x, "y": y, "z": z}
    
    def move_joint(self, joint_index: int, delta_angle: float) -> bool:
        """Move a specific joint by delta angle"""
        if joint_index < 0 or joint_index >= 6:
            self.get_logger().error(f"Invalid joint index: {joint_index}")
            return False
        
        new_angle = self.joint_angles[joint_index] + delta_angle
        self.joint_angles[joint_index] = self.clamp_joint_angle(joint_index, new_angle)
        
        # Update end effector position
        self.end_effector_position = self.forward_kinematics()
        
        self.status = "moving"
        self.get_logger().info(f"Joint {joint_index+1} moved to {self.joint_angles[joint_index]:.1f}°")
        
        return True
    
    def move_joints(self, joint_deltas: list) -> bool:
        """Move multiple joints simultaneously"""
        if len(joint_deltas) != 6:
            self.get_logger().error(f"Expected 6 joint deltas, got {len(joint_deltas)}")
            return False
        
        for i, delta in enumerate(joint_deltas):
            if delta != 0:
                new_angle = self.joint_angles[i] + delta
                self.joint_angles[i] = self.clamp_joint_angle(i, new_angle)
        
        # Update end effector position
        self.end_effector_position = self.forward_kinematics()
        
        self.status = "moving"
        self.get_logger().info(f"Joints moved to: {[f'{a:.1f}°' for a in self.joint_angles]}")
        
        return True
    
    def set_joint_angles(self, angles: list) -> bool:
        """Set absolute joint angles"""
        if len(angles) != 6:
            self.get_logger().error(f"Expected 6 joint angles, got {len(angles)}")
            return False
        
        for i, angle in enumerate(angles):
            self.joint_angles[i] = self.clamp_joint_angle(i, angle)
        
        # Update end effector position
        self.end_effector_position = self.forward_kinematics()
        
        self.status = "moving"
        self.get_logger().info(f"Joint angles set to: {[f'{a:.1f}°' for a in self.joint_angles]}")
        
        return True
    
    def move_end_effector(self, dx: float, dy: float, dz: float) -> bool:
        """Move end effector by delta position (simplified inverse kinematics)"""
        # Simplified approach: adjust joints to approximate the movement
        # In a real robot, this would use proper inverse kinematics
        
        # Scale the movement
        dx *= self.position_step_size
        dy *= self.position_step_size  
        dz *= self.position_step_size
        
        # Simple heuristic movements
        if abs(dx) > 0.01 or abs(dy) > 0.01:
            # Base rotation for X/Y movement
            base_delta = math.degrees(math.atan2(dy, dx)) * 0.1
            self.move_joint(0, base_delta)
        
        if abs(dz) > 0.01:
            # Shoulder movement for Z
            shoulder_delta = dz * 10  # degrees
            self.move_joint(1, shoulder_delta)
        
        return True
    
    def stop_movement(self) -> bool:
        """Stop all movement"""
        self.status = "idle"
        self.get_logger().info("Robot arm stopped")
        return True
    
    def reset_position(self) -> bool:
        """Reset to home position"""
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.end_effector_position = self.forward_kinematics()
        self.status = "idle"
        self.get_logger().info("Robot arm reset to home position")
        return True
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands from web interface or other sources"""
        # Convert Twist to joint movements
        linear = msg.linear
        angular = msg.angular
        
        # Map linear velocities to end effector movement
        if abs(linear.x) > 0.01 or abs(linear.y) > 0.01 or abs(linear.z) > 0.01:
            self.move_end_effector(linear.x, linear.y, linear.z)
        
        # Map angular velocities to wrist movements
        if abs(angular.x) > 0.01:  # Roll
            self.move_joint(3, angular.x * self.joint_step_size)
        if abs(angular.y) > 0.01:  # Pitch  
            self.move_joint(4, angular.y * self.joint_step_size)
        if abs(angular.z) > 0.01:  # Yaw
            self.move_joint(5, angular.z * self.joint_step_size)
    
    def joint_command_callback(self, msg: JointState):
        """Handle direct joint commands"""
        if len(msg.position) == 6:
            # Absolute positioning
            angles = [math.degrees(pos) for pos in msg.position]
            self.set_joint_angles(angles)
        elif len(msg.velocity) == 6:
            # Relative movement
            deltas = [vel * self.joint_step_size for vel in msg.velocity]
            self.move_joints(deltas)
    
    def publish_state(self):
        """Publish current robot state"""
        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_msg.position = [math.radians(angle) for angle in self.joint_angles]
        self.joint_state_pub.publish(joint_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = self.status
        self.status_pub.publish(status_msg)
        
        # Update external callback
        if self.on_state_update:
            state = {
                "position": self.end_effector_position,
                "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},  # Simplified
                "joint_angles": self.joint_angles.copy(),
                "status": self.status,
                "battery": self.battery,
                "mode": "simulation"
            }
            self.on_state_update(state)
        
        # Reset status to idle after movement
        if self.status == "moving":
            self.status = "idle"

class ROSInterface:
    """Interface for ROS 2 Robot Arm Control"""
    
    def __init__(self, mode: OperationMode = OperationMode.SIMULATION, 
                 ros_bridge_url: str = "ws://localhost:9090",
                 on_state_update: Optional[Callable] = None):
        self.mode = mode
        self.ros_bridge_url = ros_bridge_url
        self.on_state_update = on_state_update
        self.connected = False
        self.ros_node = None
        self.executor = None
        self.ros_thread = None
        
        print(f"ROSInterface initialized in {mode} mode", file=sys.stderr)
    
    async def start(self):
        """Start ROS interface"""
        try:
            # Initialize ROS 2
            if not rclpy.ok():
                rclpy.init()
            
            # Create ROS node
            self.ros_node = ROSNode(on_state_update=self.on_state_update)
            
            # Create executor
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.ros_node)
            
            # Start ROS in separate thread
            self.ros_thread = threading.Thread(target=self._run_ros, daemon=True)
            self.ros_thread.start()
            
            self.connected = True
            print("ROS 2 6-DOF Robot Arm interface started successfully", file=sys.stderr)
            
        except Exception as e:
            print(f"Failed to start ROS interface: {e}", file=sys.stderr)
            self.connected = False
    
    def _run_ros(self):
        """Run ROS executor in separate thread"""
        try:
            self.executor.spin()
        except Exception as e:
            print(f"ROS executor error: {e}", file=sys.stderr)
            self.connected = False
    
    def get_state(self) -> Dict[str, Any]:
        """Get current robot state"""
        if not self.ros_node:
            return {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                "joint_angles": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "status": "disconnected",
                "battery": 100.0,
                "mode": self.mode.value
            }
        
        return {
            "position": self.ros_node.end_effector_position.copy(),
            "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "joint_angles": self.ros_node.joint_angles.copy(),
            "status": self.ros_node.status,
            "battery": self.ros_node.battery,
            "mode": self.mode.value
        }
    
    def move_robot(self, x: float, y: float, z: float) -> bool:
        """Move robot end effector to position"""
        if not self.connected or not self.ros_node:
            print("ROS not connected", file=sys.stderr)
            return False
        
        return self.ros_node.move_end_effector(x, y, z)
    
    def move_joint(self, joint_index: int, angle: float) -> bool:
        """Move specific joint by angle (degrees)"""
        if not self.connected or not self.ros_node:
            print("ROS not connected", file=sys.stderr)
            return False
        
        return self.ros_node.move_joint(joint_index, angle)
    
    def move_joints(self, joint_deltas: list) -> bool:
        """Move multiple joints by delta angles"""
        if not self.connected or not self.ros_node:
            print("ROS not connected", file=sys.stderr)
            return False
        
        return self.ros_node.move_joints(joint_deltas)
    
    def set_joint_angles(self, angles: list) -> bool:
        """Set absolute joint angles"""
        if not self.connected or not self.ros_node:
            print("ROS not connected", file=sys.stderr)
            return False
        
        return self.ros_node.set_joint_angles(angles)
    
    def rotate_robot(self, roll: float, pitch: float, yaw: float) -> bool:
        """Rotate robot wrist"""
        if not self.connected or not self.ros_node:
            print("ROS not connected", file=sys.stderr)
            return False
        
        # Map to wrist joints (simplified)
        success = True
        if abs(roll) > 0.01:
            success &= self.ros_node.move_joint(3, roll)  # Wrist roll
        if abs(pitch) > 0.01:
            success &= self.ros_node.move_joint(4, pitch)  # Wrist pitch
        if abs(yaw) > 0.01:
            success &= self.ros_node.move_joint(5, yaw)  # Wrist yaw
        
        return success
    
    def stop_robot(self) -> bool:
        """Stop robot movement"""
        if not self.connected or not self.ros_node:
            print("ROS not connected", file=sys.stderr)
            return False
        
        return self.ros_node.stop_movement()
    
    def reset_robot(self) -> bool:
        """Reset robot to home position"""
        if not self.connected or not self.ros_node:
            print("ROS not connected", file=sys.stderr)
            return False
        
        return self.ros_node.reset_position()
    
    def set_mode(self, mode: OperationMode) -> bool:
        """Set operation mode"""
        self.mode = mode
        print(f"Mode set to {mode}", file=sys.stderr)
        return True
    
    def shutdown(self):
        """Shutdown ROS interface"""
        self.connected = False
        
        if self.executor:
            self.executor.shutdown()
        
        if self.ros_node:
            self.ros_node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        
        print("ROS interface shutdown", file=sys.stderr)
