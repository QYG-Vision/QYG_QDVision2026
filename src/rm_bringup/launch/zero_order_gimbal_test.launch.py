import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rm_bringup')
    launch_params_path = os.path.join(bringup_share, 'config', 'launch_params.yaml')
    serial_params = os.path.join(bringup_share, 'config', 'node_params', 'serial_driver_params.yaml')
    test_params = os.path.join(
        bringup_share, 'config', 'node_params', 'zero_order_gimbal_test_params.yaml')
    with open(launch_params_path, encoding='utf-8') as launch_params_file:
        launch_params = yaml.safe_load(launch_params_file) or {}

    actions = [
        Node(
            package='rm_serial_driver',
            executable='rm_serial_driver_node',
            name='serial_driver',
            output='screen',
            parameters=[
                serial_params,
                {
                    'has_rune': False,
                    'enable_mode_sync': False,
                },
            ],
        ),
        Node(
            package='rm_serial_driver',
            executable='zero_order_gimbal_test_node',
            name='zero_order_gimbal_test',
            output='screen',
            parameters=[test_params],
        ),
    ]

    if launch_params.get('foxglove_bridge', False):
        actions.append(Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 8765}],
        ))
    else:
        actions.append(LogInfo(msg=(
            'Foxglove bridge is disabled. To start it manually, run: '
            'ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765'
        )))

    return LaunchDescription(actions)
