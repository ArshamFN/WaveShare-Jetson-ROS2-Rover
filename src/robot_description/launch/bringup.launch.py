#!/usr/bin/env python3
"""
bringup.launch.py: full rover stack in one launch.

Starts, in order:
  1. robot_state_publisher   (URDF / TF tree: base_link -> laser)
  2. rover_driver_node       (motor control, heading hold, ZUPT, /imu/gz, /odom_wheel)
  3. twist_mux               (/cmd_vel + /cmd_vel_joy -> /cmd_vel_mux)
  4. teleop_twist_joy        (/joy -> /cmd_vel_joy)
  5. rplidar_ros             (/scan)
  6. rf2o_laser_odometry     (/odom + odom->base_link TF)
  7. slam_toolbox            (/map)

Velocity chain: twist_mux merges /cmd_vel (navigation, priority 10) and
/cmd_vel_joy (pendant joystick, priority 100) into /cmd_vel_mux, the only
velocity input of rover_driver_node. The Bool locks /pendant/teleop_mode (50)
and /pendant/estop (255) gate the mux; /joy is published by the pendant.

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
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    teleop_joy_params = os.path.join(pkg_share, 'config', 'teleop_joy.yaml')
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
        remappings=[('/cmd_vel', '/cmd_vel_mux')],
    )

    # ----------------------------------------------------------- 3. velocity mux
    # Sole publisher of /cmd_vel_mux. Inputs and locks are in twist_mux.yaml.
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params],
        remappings=[('cmd_vel_out', '/cmd_vel_mux')],
    )

    # -------------------------------------------------------- 4. joystick teleop
    # Consumes /joy from the pendant; no joy_node is started here.
    teleop_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[teleop_joy_params],
        remappings=[('cmd_vel', '/cmd_vel_joy')],
    )

    # ------------------------------------------------------------------ 5. LiDAR
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_share, 'launch', 'rplidar_c1_launch.py')
        )
    )

    # ------------------------------------------------------- 6. LiDAR odometry
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

    # -------------------------------------------------------------- 7. SLAM
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
        twist_mux,
        teleop_joy,
        rplidar,
        rf2o,
        slam_toolbox,
        rviz,
    ])
