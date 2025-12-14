#!/usr/bin/env python3
"""
TE Arm RViz Launch File
=======================
Launch the TE arm with RViz visualization.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('te_arm')
    
    # URDF file
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'te_arm.urdf'])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', urdf_file]), value_type=str
            )
        }]
    )
    
    # RViz
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'te_arm.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Include base launch
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'te_arm.launch.py'])
        ])
    )
    
    return LaunchDescription([
        base_launch,
        robot_state_publisher,
        rviz,
    ])

