import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rm_bringup')
    serial_params = os.path.join(bringup_share, 'config', 'node_params', 'serial_driver_params.yaml')
    test_params = os.path.join(
        bringup_share, 'config', 'node_params', 'zero_order_gimbal_test_params.yaml')

    return LaunchDescription([
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
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 8765}],
        ),
    ])
