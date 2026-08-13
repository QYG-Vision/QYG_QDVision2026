import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node_params = os.path.join(
        get_package_share_directory('rm_auto_replay'), 'config', 'node_params.yaml')

    auto_replay_node = Node(
        package='rm_auto_replay',
        executable='auto_replay_node_exec',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        arguments=['--ros-args', '--log-level', 'rm_auto_replay:=DEBUG'],
    )

    return LaunchDescription([auto_replay_node])
