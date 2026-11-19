#!/usr/bin/env python3
"""仅限无硬件视频配置：临时卸载视频组件，验证输入失效和恢复，并恢复原参数。"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grace', type=float, default=1.0, help='断流后允许处理尾帧的秒数')
    parser.add_argument('--gap', type=float, default=3.0, help='卸载视频后的总观察秒数')
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.grace, args.gap)) or args.gap <= args.grace:
        parser.error('必须满足 gap > grace > 0，且两者均有限')

    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
    from rcl_interfaces.msg import Parameter as ParameterMsg
    from rcl_interfaces.srv import GetParameters
    from rm_interfaces.msg import Armors, GimbalCmd, SerialReceiveData, Target

    rclpy.init()
    node = rclpy.create_node('video_recovery_check')
    phase = 'warmup'
    gap_start = None
    results = {name: {'armors': 0, 'tracking': 0, 'not_tracking': 0,
                      'active_commands': 0, 'idle_commands': 0, 'serial': 0}
               for name in ('warmup', 'gap_grace', 'gap', 'recovery')}
    first_idle_seconds = None

    def receive(kind, message):
        nonlocal first_idle_seconds
        current = phase
        if phase == 'gap' and time.monotonic() - gap_start < args.grace:
            current = 'gap_grace'
        item = results[current]
        if kind == 'target':
            item['tracking' if message.tracking else 'not_tracking'] += 1
        elif kind == 'command':
            idle = (message.distance < 0 and not message.fire_advice and message.id == '0')
            item['idle_commands' if idle else 'active_commands'] += 1
            if phase == 'gap' and idle and first_idle_seconds is None:
                first_idle_seconds = time.monotonic() - gap_start
        else:
            item[kind] += 1

    for kind, message, topic in (
        ('armors', Armors, '/armor_solver/armors'),
        ('target', Target, '/armor_solver/target'),
        ('command', GimbalCmd, '/armor_solver/cmd_gimbal'),
        ('serial', SerialReceiveData, '/serial/receive'),
    ):
        node.create_subscription(message, topic,
                                 lambda msg, key=kind: receive(key, msg), qos_profile_sensor_data)

    def observe(seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not rclpy.ok():
            raise RuntimeError('ROS 上下文已停止')

    def call(service_type, service, request):
        client = node.create_client(service_type, service)
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError(f'服务不可用：{service}')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done():
                raise RuntimeError(f'服务超时：{service}')
            response = future.result()
            if response is None:
                raise RuntimeError(f'服务失败：{service}')
            return response
        finally:
            node.destroy_client(client)

    reload_request = None
    unloaded = False
    restored = False
    error = None
    try:
        listed = call(ListNodes, '/rm_container/_container/list_nodes', ListNodes.Request())
        names = list(listed.full_node_names)
        if ('/video_player' not in names or '/virtual_serial' not in names
                or any(name in names for name in ('/camera_driver', '/serial_driver'))):
            raise RuntimeError('只允许视频＋虚拟串口配置，不修改真实硬件输入')
        if names.count('/video_player') != 1:
            raise RuntimeError('必须恰有一个 video_player 组件')
        request = GetParameters.Request()
        request.names = ['path', 'camera_info_url', 'frame_id', 'frame_rate',
                         'start_frame', 'keep_looping']
        parameters = call(GetParameters, '/video_player/get_parameters', request)
        if any(value.type == 0 for value in parameters.values):
            raise RuntimeError('无法读取视频原参数，取消卸载')
        reload_request = LoadNode.Request()
        reload_request.package_name = 'hik_camera'
        reload_request.plugin_name = 'qd::camera_driver::VideoPlayerNode'
        reload_request.node_name = 'video_player'
        reload_request.node_namespace = '/'
        reload_request.parameters = [ParameterMsg(name=name, value=value)
                                     for name, value in zip(request.names, parameters.values)]
        reload_request.extra_arguments = [
            Parameter('use_intra_process_comms', value=True).to_parameter_msg()]

        print('观察正常输入 3 秒，然后临时卸载视频组件；虚拟串口保持运行。', flush=True)
        observe(3)
        if results['warmup']['tracking'] < 2 or results['warmup']['active_commands'] < 2:
            raise RuntimeError('尚未建立有效跟踪，不执行断流测试')
        unload = UnloadNode.Request()
        unload.unique_id = listed.unique_ids[names.index('/video_player')]
        response = call(UnloadNode, '/rm_container/_container/unload_node', unload)
        if not response.success:
            raise RuntimeError(f'视频卸载失败：{response.error_message}')
        unloaded = True
        phase = 'gap'
        gap_start = time.monotonic()
        print(f'视频已卸载，观察 {args.gap:g} 秒；前 {args.grace:g} 秒为尾帧宽限。', flush=True)
        observe(args.gap)
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or type(exc).__name__
    finally:
        if unloaded and rclpy.ok():
            try:
                response = call(LoadNode, '/rm_container/_container/load_node', reload_request)
                if not response.success:
                    raise RuntimeError(response.error_message)
                restored = True
                phase = 'recovery'
                print('视频已按原参数恢复，观察重建跟踪 5 秒。', flush=True)
                observe(5)
            except (Exception, KeyboardInterrupt) as exc:
                error = f'{error or ""} 恢复失败：{exc}'.strip()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    passed = (error is None and restored
              and results['gap']['armors'] == 0
              and results['gap']['active_commands'] == 0
              and results['gap']['idle_commands'] >= 2
              and results['gap']['serial'] >= 2
              and results['gap_grace']['not_tracking'] + results['gap']['not_tracking'] >= 1
              and results['recovery']['tracking'] >= 2
              and results['recovery']['active_commands'] >= 2)
    report = {'passed': passed, 'scope': 'video_component_unload_reload_virtual_serial_only',
              'grace_seconds': args.grace, 'gap_seconds': args.gap,
              'restored': restored, 'first_idle_seconds_after_unload': first_idle_seconds,
              'phases': results, 'error': error}
    directory = Path(__file__).resolve().parents[2] / 'record-test'
    directory.mkdir(exist_ok=True)
    path = directory / f'stream-recovery-{datetime.now():%Y%m%d-%H%M%S-%f}.json'
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f'报告：{path}')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
