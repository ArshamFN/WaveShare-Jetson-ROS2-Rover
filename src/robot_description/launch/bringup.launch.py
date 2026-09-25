#!/usr/bin/env python3
"""
bringup.launch.py — full rover stack in one launch.

Starts, in order:
  1. robot_state_publisher   (URDF / TF tree: base_link -> laser)
  2. rover_driver_node       (motor control, heading hold, ZUPT, /imu/gz, /odom_wheel)
  3. rplidar_ros             (/scan)
  4. rf2o_laser_odometry     (/odom + odom->base_link TF)
  5. slam_toolbox            (/map)

NOT included (keep as a separate terminal):
  - teleop_twist_keyboard    (needs keyboard focus)

Usage:
    ros2 launch robot_description bringup.launch.py
    ros2 launch robot_description bringup.launch.py use_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    rplidar_share = get_package_share_directory('rplidar_ros')

    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rf2o_params = os.path.join(pkg_share, 'config', 'rf2o_params.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf')

    use_rviz = LaunchConfiguration('use_rviz')

    # ---------------------------------------------------------------- 1. TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': False,
        }],
    )

    # ------------------------------------------------------------- 2. MFD driver
    # Publishes /odom_wheel and /imu/gz. Does NOT broadcast odom->base_link;
    # RF2O owns that transform as of session 012.
    rover_driver = Node(
        package='rover_driver',
        executable='rover_driver_node',
        name='rover_driver_node',
        output='screen',
    )

    # ------------------------------------------------------------------ 3. LiDAR
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_share, 'launch', 'rplidar_c1_launch.py')
        )
    )

    # ------------------------------------------------------- 4. LiDAR odometry
    # Delayed 3s: RF2O blocks on the first scan, so let the LiDAR spin up first.
    rf2o = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[rf2o_params],
            )
        ],
    )

    # -------------------------------------------------------------- 5. SLAM
    # Delayed 5s: needs a live odom->base_link from RF2O before its first scan.
    slam_toolbox = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params],
            )
        ],
    )

    # ------------------------------------------------------------ optional rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 alongside the stack.',
        ),
        robot_state_publisher,
        rover_driver,
        rplidar,
        rf2o,
        slam_toolbox,
        rviz,
    ])
