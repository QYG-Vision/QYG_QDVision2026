#!/usr/bin/env python3
"""
gimbal2camera pitch 自动标定节点

原理：
  TF 链： camera_optical_frame → camera_link → gimbal_link → odom
  gimbal2camera pitch 误差会导致跟踪目标的 z 坐标（在 odom 中）随云台 pitch
  变化而波动。目标在平地上，odom 中的 z 应为常数。
  本节点订阅 /armor_solver/armors（已在 odom 中），通过 TF 反变换回
  camera_optical_frame，再用候选 pitch 值重算到 odom 的变换，
  找到使 z 方差最小的 pitch。

使用方法：
  ros2 run rm_bringup calibrate_gimbal2camera_pitch.py
  ros2 run rm_bringup calibrate_gimbal2camera_pitch.py --ros-args -p sample_duration:=60.0
"""

import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf2_geometry_msgs  # noqa: F401  Registers PoseStamped transforms with tf2.
import tf2_ros
import tf_transformations
from geometry_msgs.msg import PoseStamped
from rm_interfaces.msg import Armors


class Gimbal2CameraPitchCalibrator(Node):
    def __init__(self):
        super().__init__('gimbal2camera_pitch_calibrator')

        self.declare_parameter('sample_duration', 30.0)
        self.declare_parameter('gimbal2camera_x', 0.0461892)
        self.declare_parameter('gimbal2camera_y', 0.0)
        self.declare_parameter('gimbal2camera_z', -0.0603874)
        self.declare_parameter('gimbal2camera_pitch', 0.091)
        self.declare_parameter('pitch_min', -0.3)
        self.declare_parameter('pitch_max', 0.3)
        self.declare_parameter('pitch_step', 0.001)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.armors_sub = self.create_subscription(
            Armors, 'armor_solver/armors', self.armors_callback, qos_profile_sensor_data)

        self.samples = []
        self.start_time = None
        self.done = False

        self.timer = self.create_timer(1.0, self.check_collection_done)

        # 预计算固定变换
        self._init_fixed_transforms()

        sample_duration = self.get_parameter('sample_duration').value
        self.get_logger().info(
            f'Pitch calibrator started. Collecting data for {sample_duration:.0f} seconds...')

    def _init_fixed_transforms(self):
        # camera_link → camera_optical_frame (URDF camera_optical_joint)
        # origin rpy [-pi/2, 0, -pi/2], xyz [0,0,0]
        # TF/URDF 中该旋转表示 child(camera_optical_frame) 在 parent(camera_link) 下的姿态。
        # 因此点从 optical 变到 camera_link 时直接使用该矩阵。
        self.R_camera_optical_to_camera_link = tf_transformations.euler_matrix(
            -math.pi / 2.0, 0.0, -math.pi / 2.0)[:3, :3]

    def armors_callback(self, msg: Armors):
        if self.done:
            return
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        if len(msg.armors) == 0:
            return

        # 查找此时间戳对应的 gimbal 位姿，查不到则跳过
        try:
            t = self.tf_buffer.lookup_transform(
                'odom', 'gimbal_link', msg.header.stamp,
                timeout=Duration(seconds=0.05))
        except Exception:
            return

        gimbal_pos = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ])
        gimbal_quat = np.array([
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ])

        # armor_solver/armors 的 pose 已在 odom 中，用 TF 反变换回 camera_optical_frame
        armor_poses_camopt = []
        for armor in msg.armors:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = armor.pose
            try:
                ps_camopt = self.tf_buffer.transform(
                    ps, 'camera_optical_frame',
                    timeout=Duration(seconds=0.05))
                armor_poses_camopt.append(ps_camopt.pose)
            except Exception:
                pass

        if not armor_poses_camopt:
            return

        self.samples.append({
            'stamp': msg.header.stamp,
            'armor_poses': armor_poses_camopt,
            'gimbal_pos': gimbal_pos,
            'gimbal_quat': gimbal_quat,
        })

    def check_collection_done(self):
        if self.done or self.start_time is None:
            return
        elapsed_ns = (self.get_clock().now() - self.start_time).nanoseconds
        if elapsed_ns / 1e9 >= self.get_parameter('sample_duration').value:
            self.done = True
            self.get_logger().info(
                f'Collection finished. {len(self.samples)} valid samples.')
            self.compute_optimal_pitch()

    def compute_optimal_pitch(self):
        if len(self.samples) < 10:
            self.get_logger().error(
                f'Too few samples ({len(self.samples)}) for calibration. Need at least 10.')
            rclpy.shutdown()
            return

        x = self.get_parameter('gimbal2camera_x').value
        y = self.get_parameter('gimbal2camera_y').value
        z = self.get_parameter('gimbal2camera_z').value
        t_g2c = np.array([x, y, z])

        pitch_min = self.get_parameter('pitch_min').value
        pitch_max = self.get_parameter('pitch_max').value
        pitch_step = self.get_parameter('pitch_step').value
        if pitch_step <= 0.0:
            self.get_logger().error(f'Invalid pitch_step {pitch_step:.6f}; it must be > 0.')
            rclpy.shutdown()
            return
        if pitch_max < pitch_min:
            self.get_logger().error(
                f'Invalid pitch range [{pitch_min:.6f}, {pitch_max:.6f}]; pitch_max must be >= pitch_min.')
            rclpy.shutdown()
            return

        # 预提取多帧数据
        gimbal_mats = []
        for s in self.samples:
            R_og = tf_transformations.quaternion_matrix(s['gimbal_quat'])[:3, :3]
            t_og = s['gimbal_pos']
            gimbal_mats.append((R_og, t_og))

        # 预提取所有 armor 位姿 (camera_optical_frame)
        # 展平: 每个 armor 对应一个 (camopt_pos, sample_idx)
        all_armor_pts = []
        for i, s in enumerate(self.samples):
            for pose in s['armor_poses']:
                p = np.array([
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ])
                all_armor_pts.append((p, i))

        total_pts = len(all_armor_pts)
        if total_pts < 10:
            self.get_logger().error(
                f'Too few armor points ({total_pts}) for calibration.')
            rclpy.shutdown()
            return

        self.get_logger().info(
            f'Sweeping pitch over [{pitch_min:.4f}, {pitch_max:.4f}] step '
            f'{pitch_step:.4f} rad ({int((pitch_max - pitch_min) / pitch_step) + 1} steps, '
            f'{total_pts} armor points)...')

        best_pitch = 0.0
        best_variance = float('inf')
        best_z_values = None
        best_step_idx = -1

        n_steps = int(round((pitch_max - pitch_min) / pitch_step)) + 1

        def evaluate_pitch(pitch):
            # URDF/TF 的 gimbal_link -> camera_link joint 表示 camera_link 在 gimbal_link 下的姿态。
            # 点从 camera_link 变到 gimbal_link 时使用该正向矩阵和平移。
            R_gimbal_to_camera = tf_transformations.euler_matrix(0.0, pitch, 0.0)[:3, :3]
            z_values = np.empty(total_pts)
            for j, (p_opt, sample_idx) in enumerate(all_armor_pts):
                R_og, t_og = gimbal_mats[sample_idx]
                p_camera_link = self.R_camera_optical_to_camera_link @ p_opt
                p_gimbal = R_gimbal_to_camera @ p_camera_link + t_g2c
                p_odom = R_og @ p_gimbal + t_og
                z_values[j] = p_odom[2]

            variance = np.var(z_values)
            return variance, z_values

        for step_idx in range(n_steps):
            pitch = pitch_min + step_idx * pitch_step
            variance, z_values = evaluate_pitch(pitch)

            if variance < best_variance:
                best_variance = variance
                best_pitch = pitch
                best_z_values = z_values
                best_step_idx = step_idx

            if step_idx % 50 == 0 or step_idx == n_steps - 1:
                self.get_logger().debug(
                    f'  pitch={pitch:.4f} rad ({math.degrees(pitch):.2f} deg), '
                    f'z_var={variance:.6f}, z_mean={np.mean(z_values):.4f}')

        # 输出结果
        self.get_logger().info('=' * 60)
        self.get_logger().info('Calibration result:')
        if n_steps <= 1:
            self.get_logger().warning(
                'Only one pitch value was evaluated; calibration cannot check convergence.')
        elif best_step_idx in (0, n_steps - 1):
            boundary = 'minimum' if best_step_idx == 0 else 'maximum'
            self.get_logger().warning(
                f'Best pitch is on the search {boundary} boundary; calibration did not converge '
                'to an interior minimum. Check pitch range, sample quality, and TF direction.')
        self.get_logger().info(
            f'  Optimal gimbal2camera pitch: {best_pitch:.6f} rad '
            f'({math.degrees(best_pitch):.4f} deg)')
        self.get_logger().info(
            f'  Z variance at optimum:        {best_variance:.6f}')
        if best_z_values is not None:
            self.get_logger().info(
                f'  Z mean at optimum:           {np.mean(best_z_values):.4f}')
            self.get_logger().info(
                f'  Z std at optimum:            {np.std(best_z_values):.4f}')
        current_pitch = self.get_parameter('gimbal2camera_pitch').value
        current_variance, _ = evaluate_pitch(current_pitch)
        self.get_logger().info(
            f'  Current config pitch:         {current_pitch:.6f} rad '
            f'({math.degrees(current_pitch):.4f} deg)')
        self.get_logger().info(
            f'  Z variance at current config: {current_variance:.6f}')
        self.get_logger().info('=' * 60)

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Gimbal2CameraPitchCalibrator()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
