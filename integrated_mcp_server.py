#!/usr/bin/env python3
"""
Integrated MCP Server with ROS for Robot Control
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
    MCP Server for Robot Control with ROS Integration
    """
    def __init__(self, mode: OperationMode = OperationMode.SIMULATION, 
                 ros_bridge_url: str = "ws://localhost:9090"):
        self.mode = mode
        self.app = FastMCP(name="Robot Control MCP Server")
        
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
        print(f"ROS State Update: {json.dumps(state, indent=2)}", file=sys.stderr)
        
    def register_http_endpoints(self):
        """Register HTTP endpoints for web interface"""
        
        @self.http_app.get("/status")
        async def get_status_http():
            """HTTP endpoint to get robot status"""
            return self.ros_interface.get_state()
        
    def register_tools(self):
        """Register all MCP tools"""
        
        @self.app.tool(description="Get current robot status and state information")
        def get_status(ctx: Context) -> Dict[str, Any]:
            """Get current robot status"""
            ctx.info("Getting robot status")
            return self.ros_interface.get_state()
        
        @self.app.tool(description="Move robot to specified position coordinates")
        def move_robot(x: float = 0.0, y: float = 0.0, z: float = 0.0, ctx: Context = None) -> Dict[str, Any]:
            """Move robot to specified position"""
            if ctx:
                ctx.info(f"Moving robot to position: x={x}, y={y}, z={z}")
            
            success = self.ros_interface.move_robot(x, y, z)
            
            if success:
                return {"status": "moving", "position": {"x": x, "y": y, "z": z}}
            else:
                raise Exception("Failed to send movement command to ROS")
        
        @self.app.tool(description="Rotate robot to specified orientation")
        def rotate_robot(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, ctx: Context = None) -> Dict[str, Any]:
            """Rotate robot to specified orientation"""
            if ctx:
                ctx.info(f"Rotating robot to orientation: roll={roll}, pitch={pitch}, yaw={yaw}")
            
            success = self.ros_interface.rotate_robot(roll, pitch, yaw)
            
            if success:
                return {"status": "rotating", "orientation": {"roll": roll, "pitch": pitch, "yaw": yaw}}
            else:
                raise Exception("Failed to send rotation command to ROS")
        
        @self.app.tool(description="Stop all robot movement immediately")
        def stop_robot(ctx: Context) -> Dict[str, Any]:
            """Stop robot movement"""
            ctx.info("Stopping robot movement")
            success = self.ros_interface.stop_robot()
            
            if success:
                return {"status": "idle"}
            else:
                raise Exception("Failed to send stop command to ROS")
        
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
    
    parser = argparse.ArgumentParser(description="MCP Robot Control Server")
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
