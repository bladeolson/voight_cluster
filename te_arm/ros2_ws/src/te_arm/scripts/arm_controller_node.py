#!/usr/bin/env python3
"""
TE Arm Controller Node
======================
ROS2 node that controls the 6-DOF robot arm via serial bridge.

Subscribes:
  - /te_arm/joint_commands (sensor_msgs/JointState) - Target joint positions
  - /te_arm/gripper_command (std_msgs/Float64) - Gripper position 0-180

Publishes:
  - /te_arm/joint_states (sensor_msgs/JointState) - Current joint positions
  - /te_arm/status (std_msgs/String) - Arm status JSON

Services:
  - /te_arm/home (std_srvs/Trigger) - Move to home position
  - /te_arm/enable (std_srvs/Trigger) - Enable servos
  - /te_arm/disable (std_srvs/Trigger) - Disable servos
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger

import asyncio
import json
import threading
from typing import Optional

# Import the bridge (adjust path as needed)
import sys
sys.path.insert(0, '/home/bladeolson/voight_cluster/te_arm')
from te_arm_bridge import TeArmBridge, ArmState


class TeArmControllerNode(Node):
    """ROS2 node for controlling the TE robot arm."""
    
    JOINT_NAMES = [
        'te_base_joint',
        'te_shoulder_joint', 
        'te_elbow_joint',
        'te_wrist_pitch_joint',
        'te_wrist_roll_joint',
        'te_gripper_joint'
    ]
    
    def __init__(self):
        super().__init__('te_arm_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('auto_detect_port', True)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.auto_detect = self.get_parameter('auto_detect_port').value
        
        # Initialize bridge
        self.bridge: Optional[TeArmBridge] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self._async_thread: Optional[threading.Thread] = None
        
        # QoS for reliable delivery
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, '/te_arm/joint_states', qos)
        self.status_pub = self.create_publisher(
            String, '/te_arm/status', qos)
        
        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            JointState, '/te_arm/joint_commands',
            self.joint_command_callback, qos)
        self.gripper_sub = self.create_subscription(
            Float64, '/te_arm/gripper_command',
            self.gripper_callback, qos)
        
        # Services
        self.home_srv = self.create_service(
            Trigger, '/te_arm/home', self.home_callback)
        self.enable_srv = self.create_service(
            Trigger, '/te_arm/enable', self.enable_callback)
        self.disable_srv = self.create_service(
            Trigger, '/te_arm/disable', self.disable_callback)
        
        # Timer for publishing state
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_state)
        
        # Start async event loop in background thread
        self._start_async_loop()
        
        # Connect to arm
        self._run_async(self._connect())
        
        self.get_logger().info('TE Arm Controller initialized')
    
    def _start_async_loop(self):
        """Start asyncio event loop in background thread."""
        def run_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_forever()
        
        self._async_thread = threading.Thread(target=run_loop, daemon=True)
        self._async_thread.start()
    
    def _run_async(self, coro):
        """Run an async coroutine from sync context."""
        if self.loop is None:
            return None
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        try:
            return future.result(timeout=5.0)
        except Exception as e:
            self.get_logger().error(f'Async error: {e}')
            return None
    
    async def _connect(self):
        """Connect to the arm."""
        self.bridge = TeArmBridge(
            port=self.serial_port,
            baudrate=self.baudrate
        )
        
        if await self.bridge.connect(auto_detect=self.auto_detect):
            self.get_logger().info(
                f'Connected to TE arm at {self.bridge.port}')
        else:
            self.get_logger().error('Failed to connect to TE arm')
    
    def publish_state(self):
        """Publish current joint states."""
        if self.bridge is None or not self.bridge.state.connected:
            return
        
        # Publish joint state
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        # Convert degrees to radians for ROS convention
        import math
        msg.position = [math.radians(j) for j in self.bridge.state.joints]
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        self.joint_state_pub.publish(msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            'connected': self.bridge.state.connected,
            'enabled': self.bridge.state.enabled,
            'joints_deg': self.bridge.state.joints
        })
        self.status_pub.publish(status_msg)
    
    def joint_command_callback(self, msg: JointState):
        """Handle incoming joint commands."""
        if self.bridge is None:
            return
        
        if len(msg.position) < 6:
            self.get_logger().warn('Joint command must have 6 positions')
            return
        
        # Convert radians to degrees
        import math
        joints_deg = [math.degrees(p) for p in msg.position[:6]]
        
        self._run_async(self.bridge.move_joints(joints_deg))
    
    def gripper_callback(self, msg: Float64):
        """Handle gripper command."""
        if self.bridge is None:
            return
        
        value = int(max(0, min(180, msg.data)))
        self._run_async(self.bridge.set_gripper(value))
    
    def home_callback(self, request, response):
        """Home service callback."""
        if self.bridge is None:
            response.success = False
            response.message = 'Not connected'
            return response
        
        result = self._run_async(self.bridge.home())
        response.success = result or False
        response.message = 'Moved to home' if result else 'Failed'
        return response
    
    def enable_callback(self, request, response):
        """Enable service callback."""
        if self.bridge is None:
            response.success = False
            response.message = 'Not connected'
            return response
        
        result = self._run_async(self.bridge.enable())
        response.success = result or False
        response.message = 'Servos enabled' if result else 'Failed'
        return response
    
    def disable_callback(self, request, response):
        """Disable service callback."""
        if self.bridge is None:
            response.success = False
            response.message = 'Not connected'
            return response
        
        result = self._run_async(self.bridge.disable())
        response.success = result or False
        response.message = 'Servos disabled' if result else 'Failed'
        return response
    
    def destroy_node(self):
        """Clean up on shutdown."""
        if self.bridge:
            self._run_async(self.bridge.disconnect())
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeArmControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

