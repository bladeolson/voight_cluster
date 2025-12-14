#!/usr/bin/env python3
"""
TE Arm Serial Bridge
====================
Bridges between ROS2 and the Arduino-based robot arm over serial.
Runs on te.local alongside the existing FastAPI server.

This can run standalone or be imported into zen_te_server.py
"""

import asyncio
import json
import serial
import serial.tools.list_ports
from dataclasses import dataclass
from typing import Optional, Callable
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("te_arm_bridge")


@dataclass
class ArmState:
    """Current state of the robot arm."""
    joints: list[float]  # 6 joint positions in degrees
    enabled: bool = False
    connected: bool = False


class TeArmBridge:
    """
    Serial bridge to the Arduino-controlled robot arm.
    
    Usage:
        bridge = TeArmBridge()
        await bridge.connect()
        await bridge.move_joints([90, 90, 90, 90, 90, 90])
        state = await bridge.get_status()
    """
    
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout: float = 2.0
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.state = ArmState(joints=[90.0] * 6)
        self._lock = asyncio.Lock()
        self._on_state_change: Optional[Callable[[ArmState], None]] = None
    
    @staticmethod
    def find_arduino_port() -> Optional[str]:
        """Auto-detect Arduino serial port."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Common Arduino/serial adapter identifiers
            desc = port.description.lower() if port.description else ""
            if any(x in desc for x in ["arduino", "ch340", "ft232", "ftdi", "usb serial", "uart"]):
                return port.device
            if "ttyUSB" in port.device or "ttyACM" in port.device:
                return port.device
        return None
    
    async def connect(self, auto_detect: bool = True) -> bool:
        """
        Connect to the Arduino.
        Returns True if successful.
        """
        port = self.port
        
        if auto_detect:
            detected = self.find_arduino_port()
            if detected:
                port = detected
                logger.info(f"Auto-detected Arduino at {port}")
        
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            # Wait for Arduino to reset after serial connection
            await asyncio.sleep(2.0)
            
            # Clear any startup messages
            self.serial.reset_input_buffer()
            
            # Verify connection with status check
            response = await self._send_command({"cmd": "status"})
            if response and response.get("ok"):
                self.state.connected = True
                self.state.joints = response.get("joints", [90.0] * 6)
                self.state.enabled = response.get("enabled", False)
                logger.info(f"Connected to TE arm at {port}")
                return True
            else:
                logger.error(f"Arduino not responding correctly: {response}")
                return False
                
        except serial.SerialException as e:
            logger.error(f"Failed to connect to {port}: {e}")
            self.state.connected = False
            return False
    
    async def disconnect(self):
        """Disconnect from the Arduino."""
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.state.connected = False
        logger.info("Disconnected from TE arm")
    
    async def _send_command(self, cmd: dict) -> Optional[dict]:
        """
        Send a JSON command to the Arduino and wait for response.
        Thread-safe with asyncio lock.
        """
        if not self.serial or not self.serial.is_open:
            return None
        
        async with self._lock:
            try:
                # Send command
                cmd_str = json.dumps(cmd) + "\n"
                self.serial.write(cmd_str.encode())
                self.serial.flush()
                
                # Read response (blocking, but with timeout)
                loop = asyncio.get_event_loop()
                response_line = await loop.run_in_executor(
                    None, self.serial.readline
                )
                
                if response_line:
                    response = json.loads(response_line.decode().strip())
                    
                    # Update state from response
                    if "joints" in response:
                        self.state.joints = [float(j) for j in response["joints"]]
                    if "enabled" in response:
                        self.state.enabled = response["enabled"]
                    
                    if self._on_state_change:
                        self._on_state_change(self.state)
                    
                    return response
                    
            except json.JSONDecodeError as e:
                logger.error(f"Invalid JSON response: {e}")
            except serial.SerialException as e:
                logger.error(f"Serial error: {e}")
                self.state.connected = False
        
        return None
    
    # -------------------------------------------------------------------------
    # Arm Control Methods
    # -------------------------------------------------------------------------
    
    async def get_status(self) -> ArmState:
        """Get current arm status."""
        await self._send_command({"cmd": "status"})
        return self.state
    
    async def move_joints(
        self,
        joints: list[float],
        smooth: bool = True
    ) -> bool:
        """
        Move arm to specified joint positions.
        
        Args:
            joints: List of 6 joint angles in degrees [0-180]
            smooth: If True, move smoothly. If False, move immediately.
        
        Returns:
            True if command was successful.
        """
        if len(joints) != 6:
            logger.error("move_joints requires exactly 6 joint values")
            return False
        
        response = await self._send_command({
            "cmd": "move",
            "joints": [int(j) for j in joints],
            "smooth": smooth
        })
        
        return response is not None and response.get("ok", False)
    
    async def home(self) -> bool:
        """Move arm to home position."""
        response = await self._send_command({"cmd": "home"})
        return response is not None and response.get("ok", False)
    
    async def enable(self) -> bool:
        """Enable (attach) servos."""
        response = await self._send_command({"cmd": "enable"})
        return response is not None and response.get("ok", False)
    
    async def disable(self) -> bool:
        """Disable (detach) servos."""
        response = await self._send_command({"cmd": "disable"})
        return response is not None and response.get("ok", False)
    
    async def set_gripper(self, value: int) -> bool:
        """
        Set gripper position.
        
        Args:
            value: Gripper position 0-180 (0=closed, 180=open typically)
        """
        response = await self._send_command({
            "cmd": "grip",
            "value": int(value)
        })
        return response is not None and response.get("ok", False)
    
    def on_state_change(self, callback: Callable[[ArmState], None]):
        """Register a callback for state changes."""
        self._on_state_change = callback


# -----------------------------------------------------------------------------
# Standalone Test
# -----------------------------------------------------------------------------

async def main():
    """Test the arm bridge."""
    bridge = TeArmBridge()
    
    print("Connecting to TE arm...")
    if await bridge.connect():
        print(f"Connected! Current state: {bridge.state}")
        
        print("Moving to home...")
        await bridge.home()
        
        print("Testing joint move...")
        await bridge.move_joints([45, 90, 90, 90, 90, 90])
        await asyncio.sleep(1)
        
        print("Testing gripper...")
        await bridge.set_gripper(45)
        await asyncio.sleep(0.5)
        await bridge.set_gripper(135)
        
        print("Returning home...")
        await bridge.home()
        
        await bridge.disconnect()
    else:
        print("Failed to connect!")


if __name__ == "__main__":
    asyncio.run(main())

