#!/usr/bin/env python3
"""
TE Arm Launch File
==================
Launch the TE arm controller and visualization.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino connection'
    )
    
    auto_detect_arg = DeclareLaunchArgument(
        'auto_detect_port',
        default_value='true',
        description='Auto-detect Arduino port'
    )
    
    # Get package share directory for config/urdf
    pkg_share = FindPackageShare('te_arm')
    
    # Arm controller node
    arm_controller = Node(
        package='te_arm',
        executable='arm_controller_node.py',
        name='te_arm_controller',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'auto_detect_port': LaunchConfiguration('auto_detect_port'),
            'publish_rate': 10.0,
        }]
    )
    
    # Joint state publisher (bridges to /joint_states)
    joint_state_publisher = Node(
        package='te_arm',
        executable='joint_state_publisher.py',
        name='te_joint_state_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        auto_detect_arg,
        arm_controller,
        joint_state_publisher,
    ])

