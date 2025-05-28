#!/usr/bin/env python3
"""
Integrated MCP Server with ROS for 6-DOF Robot Arm Control
Supports both real and simulation scenarios with web visualization
"""

import os
import json
import asyncio
import sys
from typing import Dict, Any, Optional, List, Union
from enum import Enum

from fastmcp import FastMCP, Context
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from ros_interface import ROSInterface, OperationMode

class MCPRobotServer:
    """
    MCP Server for 6-DOF Robot Arm Control with ROS Integration
    """
    def __init__(self, mode: OperationMode = OperationMode.SIMULATION, 
                 ros_bridge_url: str = "ws://localhost:9090"):
        self.mode = mode
        self.app = FastMCP(name="6-DOF Robot Arm Control MCP Server")
        
        # Add HTTP endpoints for web interface
        self.http_app = FastAPI()
        self.http_app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        # Initialize ROS interface
        self.ros_interface = ROSInterface(
            mode=mode,
            ros_bridge_url=ros_bridge_url,
            on_state_update=self._handle_ros_state_update
        )
        
        # Register tools and HTTP endpoints
        self.register_tools()
        self.register_http_endpoints()
        
    def _handle_ros_state_update(self, state: Dict[str, Any]):
        """Handle state updates from ROS"""
        # Log state updates to stderr (can be extended for WebSocket broadcasting)
        print(f"Robot Arm State Update: {json.dumps(state, indent=2)}", file=sys.stderr)
        
    def register_http_endpoints(self):
        """Register HTTP endpoints for web interface"""
        
        @self.http_app.get("/status")
        async def get_status_http():
            """HTTP endpoint to get robot status"""
            return self.ros_interface.get_state()
        
    def register_tools(self):
        """Register all MCP tools for 6-DOF robot arm control"""
        
        @self.app.tool(description="Get current robot arm status including joint angles and end effector position")
        def get_status(ctx: Context) -> Dict[str, Any]:
            """Get current robot arm status"""
            ctx.info("Getting robot arm status")
            return self.ros_interface.get_state()
        
        @self.app.tool(description="Move a specific joint by angle in degrees. Joint indices: 0=base, 1=shoulder, 2=elbow, 3=wrist_roll, 4=wrist_pitch, 5=wrist_yaw. Angle range: ±15° per command for safety.")
        def move_joint(joint_index: int, angle: float, ctx: Context = None) -> Dict[str, Any]:
            """Move a specific joint by angle (degrees)"""
            if ctx:
                ctx.info(f"Moving joint {joint_index} by {angle}°")
            
            # Validate joint index
            if joint_index < 0 or joint_index > 5:
                raise Exception(f"Invalid joint index {joint_index}. Must be 0-5 (0=base, 1=shoulder, 2=elbow, 3=wrist_roll, 4=wrist_pitch, 5=wrist_yaw)")
            
            # Limit angle for safety
            angle = max(-30, min(30, angle))  # Clamp to ±30° per command
            
            success = self.ros_interface.move_joint(joint_index, angle)
            
            if success:
                joint_names = ["base", "shoulder", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"]
                return {"status": "moving", "joint": joint_names[joint_index], "angle": angle}
            else:
                raise Exception(f"Failed to move joint {joint_index}")
        
        @self.app.tool(description="Move multiple joints simultaneously. Provide 6 angles in degrees [base, shoulder, elbow, wrist_roll, wrist_pitch, wrist_yaw]. Each angle limited to ±30° per command.")
        def move_joints(joint_angles: List[float], ctx: Context = None) -> Dict[str, Any]:
            """Move multiple joints simultaneously"""
            if ctx:
                ctx.info(f"Moving joints: {joint_angles}")
            
            # Validate input
            if len(joint_angles) != 6:
                raise Exception(f"Expected 6 joint angles, got {len(joint_angles)}. Format: [base, shoulder, elbow, wrist_roll, wrist_pitch, wrist_yaw]")
            
            # Limit angles for safety
            limited_angles = [max(-30, min(30, angle)) for angle in joint_angles]
            
            success = self.ros_interface.move_joints(limited_angles)
            
            if success:
                return {"status": "moving", "joint_angles": limited_angles}
            else:
                raise Exception("Failed to move joints")
        
        @self.app.tool(description="Set absolute joint angles in degrees. Provide 6 angles [base, shoulder, elbow, wrist_roll, wrist_pitch, wrist_yaw]. Angles will be clamped to safe limits.")
        def set_joint_angles(joint_angles: List[float], ctx: Context = None) -> Dict[str, Any]:
            """Set absolute joint angles"""
            if ctx:
                ctx.info(f"Setting joint angles to: {joint_angles}")
            
            # Validate input
            if len(joint_angles) != 6:
                raise Exception(f"Expected 6 joint angles, got {len(joint_angles)}. Format: [base, shoulder, elbow, wrist_roll, wrist_pitch, wrist_yaw]")
            
            success = self.ros_interface.set_joint_angles(joint_angles)
            
            if success:
                return {"status": "moving", "target_angles": joint_angles}
            else:
                raise Exception("Failed to set joint angles")
        
        @self.app.tool(description="Move end effector by relative position in meters. Each coordinate limited to ±1.0m per command for safety.")
        def move_end_effector(x: float = 0.0, y: float = 0.0, z: float = 0.0, ctx: Context = None) -> Dict[str, Any]:
            """Move end effector by relative position"""
            if ctx:
                ctx.info(f"Moving end effector by: x={x}, y={y}, z={z}")
            
            # Limit movement for safety
            x = max(-1.0, min(1.0, x))
            y = max(-1.0, min(1.0, y))
            z = max(-1.0, min(1.0, z))
            
            success = self.ros_interface.move_robot(x, y, z)
            
            if success:
                return {"status": "moving", "delta_position": {"x": x, "y": y, "z": z}}
            else:
                raise Exception("Failed to move end effector")
        
        @self.app.tool(description="Rotate wrist joints (roll, pitch, yaw) in degrees. Each angle limited to ±30° per command.")
        def rotate_wrist(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, ctx: Context = None) -> Dict[str, Any]:
            """Rotate wrist joints"""
            if ctx:
                ctx.info(f"Rotating wrist: roll={roll}, pitch={pitch}, yaw={yaw}")
            
            # Limit angles for safety
            roll = max(-30, min(30, roll))
            pitch = max(-30, min(30, pitch))
            yaw = max(-30, min(30, yaw))
            
            success = self.ros_interface.rotate_robot(roll, pitch, yaw)
            
            if success:
                return {"status": "rotating", "wrist_rotation": {"roll": roll, "pitch": pitch, "yaw": yaw}}
            else:
                raise Exception("Failed to rotate wrist")
        
        @self.app.tool(description="Stop all robot arm movement immediately")
        def stop_robot(ctx: Context) -> Dict[str, Any]:
            """Stop robot arm movement"""
            ctx.info("Stopping robot arm movement")
            success = self.ros_interface.stop_robot()
            
            if success:
                return {"status": "idle"}
            else:
                raise Exception("Failed to send stop command")
        
        @self.app.tool(description="Reset robot arm to home position (all joints at 0°)")
        def reset_to_home(ctx: Context) -> Dict[str, Any]:
            """Reset robot arm to home position"""
            ctx.info("Resetting robot arm to home position")
            success = self.ros_interface.reset_robot()
            
            if success:
                return {"status": "idle", "position": "home"}
            else:
                raise Exception("Failed to reset to home position")
        
        @self.app.tool(description="Perform predefined movements: 'wave', 'reach_up', 'reach_forward', 'point_left', 'point_right'")
        def perform_gesture(gesture: str, ctx: Context = None) -> Dict[str, Any]:
            """Perform predefined gestures"""
            if ctx:
                ctx.info(f"Performing gesture: {gesture}")
            
            gestures = {
                "wave": [
                    [0, 0, -30, 0, 0, 0],    # Reach out slightly
                    [0, 0, -30, 0, 0, 45],   # Wrist up
                    [0, 0, -30, 0, 0, -45],  # Wrist down
                    [0, 0, -30, 0, 0, 45],   # Wrist up
                    [0, 0, -30, 0, 0, 0],    # Center
                ],
                "reach_up": [[0, -45, -20, 0, 30, 0]],  # More natural reaching up
                "reach_forward": [[0, 15, -60, 0, 45, 0]],  # Natural forward reach
                "point_left": [[60, 0, -30, 0, 0, 0]],  # Less extreme pointing
                "point_right": [[-60, 0, -30, 0, 0, 0]]  # Less extreme pointing
            }
            
            if gesture not in gestures:
                raise Exception(f"Unknown gesture '{gesture}'. Available: {list(gestures.keys())}")
            
            # Execute gesture sequence
            for angles in gestures[gesture]:
                success = self.ros_interface.set_joint_angles(angles)
                if not success:
                    raise Exception(f"Failed to execute gesture '{gesture}'")
                # Small delay between movements
                import time
                time.sleep(0.5)
            
            return {"status": "completed", "gesture": gesture}
        
        @self.app.tool(description="Set operation mode (real or simulation)")
        def set_mode(mode: str, ctx: Context) -> Dict[str, Any]:
            """Set operation mode (real or simulation)"""
            ctx.info(f"Setting operation mode to: {mode}")
            
            try:
                operation_mode = OperationMode(mode)
                success = self.ros_interface.set_mode(operation_mode)
                
                if success:
                    self.mode = operation_mode
                    return {"mode": operation_mode.value}
                else:
                    raise Exception(f"Failed to switch to mode: {mode}")
            except ValueError:
                raise Exception(f"Invalid mode: {mode}. Must be 'real' or 'simulation'")
    
    async def run(self, host: str = "0.0.0.0", port: int = 8000):
        """Run the MCP server"""
        # Start ROS interface
        await self.ros_interface.start()
        
        # Start HTTP server for web interface
        import uvicorn
        import threading
        
        def run_http_server():
            uvicorn.run(self.http_app, host=host, port=port+1, log_level="error")
        
        http_thread = threading.Thread(target=run_http_server, daemon=True)
        http_thread.start()
        
        print(f"HTTP server for web interface running on http://{host}:{port+1}", file=sys.stderr)
        
        # Run FastMCP server
        await self.app.run_async()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="6-DOF Robot Arm MCP Control Server")
    parser.add_argument(
        "--mode", 
        type=str, 
        choices=["real", "simulation"], 
        default="simulation",
        help="Operation mode: 'real' for physical robot, 'simulation' for simulated robot"
    )
    parser.add_argument(
        "--host", 
        type=str, 
        default="0.0.0.0",
        help="Host address to bind the server to"
    )
    parser.add_argument(
        "--port", 
        type=int, 
        default=8000,
        help="Port to run the server on"
    )
    parser.add_argument(
        "--ros-bridge", 
        type=str, 
        default="ws://localhost:9090",
        help="ROS bridge WebSocket URL"
    )
    
    args = parser.parse_args()
    
    # Create and run the MCP server
    server = MCPRobotServer(
        mode=OperationMode(args.mode),
        ros_bridge_url=args.ros_bridge
    )
    
    asyncio.run(server.run(host=args.host, port=args.port))
