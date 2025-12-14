#!/usr/bin/env python3
"""
TE Arm Joint State Publisher
============================
Standalone joint state publisher for visualization/debugging.
Republishes arm state for RViz visualization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String

import json
import math


class TeJointStatePublisher(Node):
    """Republish TE arm joint states for robot_state_publisher."""
    
    JOINT_NAMES = [
        'te_base_joint',
        'te_shoulder_joint',
        'te_elbow_joint', 
        'te_wrist_pitch_joint',
        'te_wrist_roll_joint',
        'te_gripper_joint'
    ]
    
    def __init__(self):
        super().__init__('te_joint_state_publisher')
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Subscribe to arm controller output
        self.sub = self.create_subscription(
            JointState, '/te_arm/joint_states',
            self.joint_callback, qos)
        
        # Publish to standard joint_states topic for robot_state_publisher
        self.pub = self.create_publisher(JointState, '/joint_states', qos)
        
        self.get_logger().info('TE Joint State Publisher started')
    
    def joint_callback(self, msg: JointState):
        """Forward joint states to standard topic."""
        # Just republish with correct names
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.name = self.JOINT_NAMES
        out_msg.position = list(msg.position)
        out_msg.velocity = list(msg.velocity) if msg.velocity else [0.0] * 6
        out_msg.effort = list(msg.effort) if msg.effort else [0.0] * 6
        
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeJointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

