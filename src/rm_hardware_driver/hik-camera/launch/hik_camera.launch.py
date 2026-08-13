import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
from launch.actions import TimerAction, Shutdown

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('hik_camera'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://hik_camera/config/camera_info.yaml'

    image_node  = ComposableNode(
            package='hik_camera',
            plugin='hik_camera::HikCameraNode',
            name='camera_driver',
            parameters=[params_file],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    
    container = ComposableNodeContainer(
            name='camera_detector_container',
            package='rclcpp_components',
            executable='component_container_mt',
            namespace='',
            composable_node_descriptions=[image_node],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', ],
        )
    
    hik_node = Node(
            package='hik_camera',
            executable='hik_camera_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
            
        )

    return LaunchDescription([
        container
    ])
