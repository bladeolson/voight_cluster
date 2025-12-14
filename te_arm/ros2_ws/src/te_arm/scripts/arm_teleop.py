#!/usr/bin/env python3
"""
TE Arm Teleop Node
==================
Keyboard teleoperation for the 6-DOF robot arm.

Controls:
  Joint Selection:
    1-6: Select joint to control
    
  Movement:
    UP/DOWN or W/S: Increase/decrease selected joint angle
    LEFT/RIGHT or A/D: Fine adjustment (smaller steps)
    
  Commands:
    H: Home position
    G: Toggle gripper (open/close)
    E: Enable servos
    X: Disable servos
    Q: Quit
    
  Speed:
    +/-: Increase/decrease movement speed
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

import sys
import termios
import tty
import math


class TeArmTeleopNode(Node):
    """Keyboard teleop node for TE arm."""
    
    JOINT_NAMES = [
        'Base (J1)',
        'Shoulder (J2)',
        'Elbow (J3)',
        'Wrist Pitch (J4)',
        'Wrist Roll (J5)',
        'Gripper (J6)'
    ]
    
    def __init__(self):
        super().__init__('te_arm_teleop')
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState, '/te_arm/joint_commands', qos)
        self.gripper_pub = self.create_publisher(
            Float64, '/te_arm/gripper_command', qos)
        
        # Subscribe to current state
        self.current_joints = [90.0] * 6  # degrees
        self.state_sub = self.create_subscription(
            JointState, '/te_arm/joint_states',
            self.state_callback, qos)
        
        # Service clients
        self.home_client = self.create_client(Trigger, '/te_arm/home')
        self.enable_client = self.create_client(Trigger, '/te_arm/enable')
        self.disable_client = self.create_client(Trigger, '/te_arm/disable')
        
        # Teleop state
        self.selected_joint = 0
        self.step_size = 5.0  # degrees
        self.gripper_open = False
        
        self.get_logger().info('TE Arm Teleop ready - press H for help')
    
    def state_callback(self, msg: JointState):
        """Update current joint positions."""
        # Convert from radians back to degrees
        self.current_joints = [math.degrees(p) for p in msg.position]
    
    def publish_joints(self):
        """Publish current joint command."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [math.radians(j) for j in self.current_joints]
        self.joint_pub.publish(msg)
    
    def adjust_joint(self, delta: float):
        """Adjust selected joint by delta degrees."""
        new_val = self.current_joints[self.selected_joint] + delta
        new_val = max(0.0, min(180.0, new_val))
        self.current_joints[self.selected_joint] = new_val
        self.publish_joints()
        self.print_status()
    
    def toggle_gripper(self):
        """Toggle gripper open/closed."""
        self.gripper_open = not self.gripper_open
        value = 150.0 if self.gripper_open else 30.0
        
        msg = Float64()
        msg.data = value
        self.gripper_pub.publish(msg)
        
        self.current_joints[5] = value
        state = "OPEN" if self.gripper_open else "CLOSED"
        print(f"\r\033[K  Gripper: {state}")
    
    def call_home(self):
        """Call home service."""
        if not self.home_client.wait_for_service(timeout_sec=1.0):
            print("\r\033[K  Home service not available")
            return
        
        future = self.home_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result():
            print(f"\r\033[K  {future.result().message}")
        self.current_joints = [90.0] * 6
    
    def call_enable(self):
        """Call enable service."""
        if not self.enable_client.wait_for_service(timeout_sec=1.0):
            print("\r\033[K  Enable service not available")
            return
        
        future = self.enable_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result():
            print(f"\r\033[K  {future.result().message}")
    
    def call_disable(self):
        """Call disable service."""
        if not self.disable_client.wait_for_service(timeout_sec=1.0):
            print("\r\033[K  Disable service not available")
            return
        
        future = self.disable_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result():
            print(f"\r\033[K  {future.result().message}")
    
    def print_status(self):
        """Print current status."""
        joints_str = " | ".join(
            f"{'>' if i == self.selected_joint else ' '}{self.JOINT_NAMES[i]}: {self.current_joints[i]:5.1f}°"
            for i in range(6)
        )
        print(f"\r\033[K{joints_str}", end='', flush=True)
    
    def print_help(self):
        """Print help message."""
        print("\n" + "=" * 60)
        print("TE Arm Teleop Controls")
        print("=" * 60)
        print("  1-6     : Select joint")
        print("  W/S     : Increase/decrease angle (large step)")
        print("  A/D     : Increase/decrease angle (small step)")
        print("  +/-     : Change step size")
        print("  H       : Move to home position")
        print("  G       : Toggle gripper")
        print("  E       : Enable servos")
        print("  X       : Disable servos")
        print("  Q/ESC   : Quit")
        print("=" * 60)
        print(f"  Current step size: {self.step_size}°")
        print()
        self.print_status()


def get_key():
    """Get a single keypress."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Handle arrow keys (escape sequences)
        if ch == '\x1b':
            ch2 = sys.stdin.read(2)
            if ch2 == '[A':
                return 'UP'
            elif ch2 == '[B':
                return 'DOWN'
            elif ch2 == '[C':
                return 'RIGHT'
            elif ch2 == '[D':
                return 'LEFT'
            return 'ESC'
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = TeArmTeleopNode()
    
    print("\n" + "=" * 60)
    print("  TE ARM TELEOP - 手 (VOIGHT CLUSTER)")
    print("=" * 60)
    print("  Press '?' for help, 'Q' to quit")
    print()
    node.print_status()
    
    try:
        while True:
            key = get_key()
            
            if key.lower() == 'q' or key == 'ESC':
                print("\n\nExiting teleop...")
                break
            
            elif key == '?':
                node.print_help()
            
            elif key in '123456':
                node.selected_joint = int(key) - 1
                node.print_status()
            
            elif key.lower() == 'w' or key == 'UP':
                node.adjust_joint(node.step_size)
            
            elif key.lower() == 's' or key == 'DOWN':
                node.adjust_joint(-node.step_size)
            
            elif key.lower() == 'd' or key == 'RIGHT':
                node.adjust_joint(node.step_size / 5)
            
            elif key.lower() == 'a' or key == 'LEFT':
                node.adjust_joint(-node.step_size / 5)
            
            elif key == '+' or key == '=':
                node.step_size = min(30.0, node.step_size + 1.0)
                print(f"\r\033[K  Step size: {node.step_size}°")
            
            elif key == '-':
                node.step_size = max(1.0, node.step_size - 1.0)
                print(f"\r\033[K  Step size: {node.step_size}°")
            
            elif key.lower() == 'h':
                print("\r\033[K  Moving to home...")
                node.call_home()
                node.print_status()
            
            elif key.lower() == 'g':
                node.toggle_gripper()
            
            elif key.lower() == 'e':
                node.call_enable()
            
            elif key.lower() == 'x':
                node.call_disable()
    
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

