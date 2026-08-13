#!/usr/bin/env python3
"""
动态相机 TF 发布节点
用于调试时实时调整 gimbal_link → camera_link 的 xyz 和 rpy（装配误差补偿）
使用方法：
  ros2 param set /dynamic_camera_tf x 0.1006     # 米
  ros2 param set /dynamic_camera_tf y -0.0465
  ros2 param set /dynamic_camera_tf z 0.0
  ros2 param set /dynamic_camera_tf pitch 0.02   # 弧度制
  ros2 param set /dynamic_camera_tf roll 0.0
  ros2 param set /dynamic_camera_tf yaw 0.0
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange


class DynamicCameraTfNode(Node):
    def __init__(self):
        super().__init__('dynamic_camera_tf')
        
        # 参数范围描述（step=0 表示不限制步进，支持任意精度）
        angle_range = FloatingPointRange(
            from_value=-1.5,  # 约 -90 度
            to_value=1.5,     # 约 +90 度
            step=0.0
        )
        angle_desc = ParameterDescriptor(
            description='Angle offset in radians',
            floating_point_range=[angle_range]
        )
        
        pos_range = FloatingPointRange(from_value=-0.5, to_value=0.5, step=0.0)
        pos_desc = ParameterDescriptor(
            description='Position offset in meters',
            floating_point_range=[pos_range]
        )
        
        # 声明可动态调整的参数（默认值与 urdf 中的 xyz arg 一致）
        self.declare_parameter('x', 0.0, pos_desc)
        self.declare_parameter('y', 0.0, pos_desc)
        self.declare_parameter('z', 0.0, pos_desc)
        self.declare_parameter('roll', 0.0, angle_desc)
        self.declare_parameter('pitch', 0.0, angle_desc)
        self.declare_parameter('yaw', 0.0, angle_desc)
        
        # TF 发布器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 定时发布 TF (100Hz)
        self.timer = self.create_timer(0.001, self.publish_tf)
        
        self.get_logger().info('Dynamic Camera TF node started. Use ros2 param set to adjust pitch/roll/yaw.')

    def publish_tf(self):
        # 获取当前参数值
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        roll = self.get_parameter('roll').value
        pitch = self.get_parameter('pitch').value
        yaw = self.get_parameter('yaw').value
        
        # 构建变换
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'gimbal_link'
        t.child_frame_id = 'camera_link'
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicCameraTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


