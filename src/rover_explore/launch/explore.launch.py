"""Frontier exploration: an explorer plus the map autosave node.

explorer:=rover (default) runs the rover's own explorer node; explorer:=explore_lite
runs explore_lite instead, kept as the baseline. Bringup and Nav2 must already
be running. The autosave node starts first and the explorer 3 s later, so an
autosave node that fails at startup shuts the launch down before exploration
can send a goal. If either node exits, the whole launch shuts down.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

EXPLORERS = {
    'rover': ('rover_explore', 'rover_explorer', 'rover_explorer'),
    'explore_lite': ('explore_lite', 'explore', 'explore_node'),
}


def nodes(context, params):
    choice = LaunchConfiguration('explorer').perform(context)
    if choice not in EXPLORERS:
        raise RuntimeError(f'explorer must be one of {sorted(EXPLORERS)}, got {choice!r}')
    package, executable, name = EXPLORERS[choice]

    autosave = Node(
        package='rover_explore',
        executable='explore_autosave',
        name='explore_autosave',
        output='screen',
        parameters=[params],
        on_exit=Shutdown(reason='explore_autosave exited'),
    )

    explorer = Node(
        package=package,
        executable=executable,
        name=name,
        output='screen',
        parameters=[params],
        on_exit=Shutdown(reason=f'{name} exited'),
    )

    return [autosave, TimerAction(period=3.0, actions=[explorer])]


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('rover_explore'), 'config', 'explore_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'explorer', default_value='rover',
            description='rover (the rover_explorer node) or explore_lite (baseline)'),
        OpaqueFunction(function=nodes, args=[params]),
    ])
