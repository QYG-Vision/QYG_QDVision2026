#!/usr/bin/env python3
"""只订阅视频自瞄链路的话题，检查功能数据流并保存验收结果。"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time


def finite(*values):
    """检查参与验收的数值是否全部有限。"""
    return all(math.isfinite(value) for value in values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0, help="观察秒数，默认 20")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration 必须是有限正数")

    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from rm_interfaces.msg import Armors, BallisticDebug, GimbalCmd, SerialReceiveData, Target
    except ImportError as error:
        parser.exit(2, f"无法导入 ROS 消息：{error}\n请在容器内先 source /ros_ws/install/setup.bash。\n")

    checks = [
        ("图像输入", "/image_raw", Image,
         lambda msg: msg.width > 0 and msg.height > 0),
        ("虚拟姿态与弹速", "/serial/receive", SerialReceiveData,
         lambda msg: finite(msg.roll, msg.pitch, msg.yaw, msg.bullet_speed)
         and msg.bullet_speed > 0),
        ("检测与坐标转换", "/armor_solver/armors", Armors,
         lambda msg: bool(msg.armors) and msg.header.frame_id == "odom"),
        ("目标跟踪", "/armor_solver/target", Target,
         lambda msg: msg.tracking
         and finite(msg.position.x, msg.position.y, msg.position.z)),
        ("云台指令输出", "/armor_solver/cmd_gimbal", GimbalCmd,
         lambda msg: finite(msg.yaw, msg.pitch, msg.distance, msg.bullet_speed)
         and msg.distance > 0 and msg.bullet_speed > 0),
        ("弹道补偿", "/armor_solver/ballistic_debug", BallisticDebug,
         lambda msg: msg.ballistic_enabled and msg.ballistic_success
         and finite(msg.geometric_pitch, msg.compensated_pitch, msg.delta_pitch)),
    ]
    results = {
        topic: {"label": label, "received": 0, "valid": 0}
        for label, topic, _, _ in checks
    }

    def make_callback(topic, predicate):
        def callback(message):
            item = results[topic]
            item["received"] += 1
            if predicate(message):
                item["valid"] += 1
        return callback

    rclpy.init()
    node = rclpy.create_node("video_pipeline_check")
    for _, topic, message_type, predicate in checks:
        node.create_subscription(
            message_type, topic, make_callback(topic, predicate), qos_profile_sensor_data
        )

    print(f"观察 {args.duration:g} 秒；只接收话题，不发送控制命令。", flush=True)
    started = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    for topic, item in results.items():
        item["passed"] = item["valid"] >= 2
        status = "通过" if item["passed"] else "未通过"
        print(f"[{status}] {item['label']}：收到 {item['received']} 条，"
              f"满足条件 {item['valid']} 条 ({topic})")

    passed = not interrupted and all(item["passed"] for item in results.values())
    report = {
        "scope": "video_and_virtual_serial_to_gimbal_command",
        "duration_seconds": round(time.monotonic() - started, 2),
        "interrupted": interrupted,
        "passed": passed,
        "checks": results,
    }
    output_dir = Path(__file__).resolve().parents[2] / "record-test"
    output_dir.mkdir(exist_ok=True)
    output = output_dir / f"pipeline-check-{datetime.now():%Y%m%d-%H%M%S-%f}.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"报告：{output}")
    if passed:
        print("视频与虚拟电控输入到云台指令输出的功能链路检查通过。")
    else:
        print("尚未完成全部验收；未通过项也可能是当前视频没有可跟踪目标。")
    print("此检查不评价瞄准精度、实时性能、串口发送或真实云台响应。")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
