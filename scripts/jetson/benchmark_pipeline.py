#!/usr/bin/env python3
"""订阅相同视频链路，记录实际处理频率和带图像时间戳的延迟。"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True, help="本次环境名称，例如 cpu-baseline")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--warmup", type=float, default=3.0)
    parser.add_argument("--skip-images", action="store_true", help="不订阅大图像，减少观测开销")
    parser.add_argument("--progress-every", type=float, default=30.0, help="分段统计与进度间隔，秒")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration 必须是有限正数")
    if not math.isfinite(args.warmup) or args.warmup < 0:
        parser.error("--warmup 必须是有限非负数")
    if not math.isfinite(args.progress_every) or args.progress_every <= 0:
        parser.error("--progress-every 必须是有限正数")

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from rm_interfaces.msg import Armors, GimbalCmd, Target

    rclpy.init()
    node = rclpy.create_node("pipeline_benchmark")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    counts = {"images": 0, "armors": 0, "targets": 0, "tracking": 0, "commands": 0}
    delays = {"armors": [], "targets": []}
    measuring = False
    last_arrival = {key: None for key in delays}
    max_gaps = {key: 0.0 for key in delays}
    last_stamp = {key: None for key in delays}
    non_increasing_stamps = {key: 0 for key in delays}
    windows = []
    resources = []

    def receive(kind, message):
        if not measuring:
            return
        counts[kind] += 1
        if kind == "targets" and message.tracking:
            counts["tracking"] += 1
        if kind in delays:
            arrival = time.monotonic()
            if last_arrival[kind] is not None:
                max_gaps[kind] = max(max_gaps[kind], arrival - last_arrival[kind])
            last_arrival[kind] = arrival
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if last_stamp[kind] is not None and stamp_ns <= last_stamp[kind]:
                non_increasing_stamps[kind] += 1
            last_stamp[kind] = stamp_ns
            age = (node.get_clock().now().nanoseconds - stamp_ns) / 1_000_000
            if stamp_ns > 0 and math.isfinite(age):
                delays[kind].append(age)

    for kind, message_type, topic in (
        ("images", Image, "/image_raw"),
        ("armors", Armors, "/armor_solver/armors"),
        ("targets", Target, "/armor_solver/target"),
        ("commands", GimbalCmd, "/armor_solver/cmd_gimbal"),
    ):
        if kind == "images" and args.skip_images:
            continue
        node.create_subscription(message_type, topic,
                                 lambda message, key=kind: receive(key, message), qos)

    def sample_resources(elapsed):
        """记录组件进程的主机 RSS/Swap；不把它等同于 GPU 总内存占用。"""
        processes = []
        for path in Path('/proc').glob('[0-9]*/comm'):
            try:
                if not path.read_text().startswith('component_conta'):
                    continue
                status = dict(line.split(':', 1) for line in
                              path.with_name('status').read_text().splitlines() if ':' in line)
                processes.append({'pid': int(path.parent.name),
                                  'rss_kib': int(status['VmRSS'].split()[0]),
                                  'swap_kib': int(status.get('VmSwap', '0 kB').split()[0])})
            except (OSError, ValueError, KeyError):
                continue
        resources.append({'elapsed_seconds': round(elapsed, 3), 'processes': processes})

    print(f"{args.label}: 预热 {args.warmup:g} 秒，测量 {args.duration:g} 秒。", flush=True)
    interrupted = False
    try:
        deadline = time.monotonic() + args.warmup
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        measuring = True
        started = time.monotonic()
        window_started = started
        window_counts = counts.copy()
        sample_resources(0)
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - window_started >= args.progress_every:
                duration = now - window_started
                rates = {key: round((counts[key] - window_counts[key]) / duration, 3)
                         for key in counts if key != 'tracking'}
                windows.append({'end_seconds': round(now - started, 3),
                                'duration_seconds': round(duration, 3), 'rates_hz': rates})
                sample_resources(now - started)
                print(f"已观察 {now - started:.0f}s，本段处理 {rates['armors']:.2f}Hz，"
                      f"跟踪 {rates['targets']:.2f}Hz。", flush=True)
                window_started = now
                window_counts = counts.copy()
        elapsed = time.monotonic() - started
        remainder = time.monotonic() - window_started
        if remainder > 0.1:
            windows.append({'end_seconds': round(elapsed, 3),
                            'duration_seconds': round(remainder, 3),
                            'rates_hz': {key: round((counts[key] - window_counts[key]) / remainder, 3)
                                         for key in counts if key != 'tracking'}})
        sample_resources(elapsed)
    except KeyboardInterrupt:
        interrupted = True
        elapsed = 0.0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def summarize(values):
        if not values:
            return None
        ordered = sorted(values)
        return {"samples": len(values), "median": round(statistics.median(values), 3),
                "p95": round(ordered[int((len(values) - 1) * 0.95)], 3),
                "max": round(ordered[-1], 3)}

    report = {
        "label": args.label, "duration_seconds": round(elapsed, 3),
        "interrupted": interrupted, "received": counts,
        "rates_hz": {key: round(value / elapsed, 3) if elapsed else 0
                     for key, value in counts.items() if key != "tracking"},
        "image_to_armors_ms": summarize(delays["armors"]),
        "image_to_target_ms": summarize(delays["targets"]),
        "subscribed_images": not args.skip_images,
        "max_result_gap_ms": {key: round(value * 1000, 3) for key, value in max_gaps.items()},
        "non_increasing_result_stamps": non_increasing_stamps,
        "windows": windows,
        "component_resources": resources,
        "note": "延迟含消息传输；images 是外部观测者接收率，可能因大图像丢帧而低于进程内处理率；指令频率不是模型 FPS，不代表实际相机或云台性能。",
    }
    directory = Path(__file__).resolve().parents[2] / "record-test"
    directory.mkdir(exist_ok=True)
    path = directory / f"benchmark-{datetime.now():%Y%m%d-%H%M%S-%f}.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"报告：{path}")
    return 0 if not interrupted and counts["armors"] >= 2 else 1


if __name__ == "__main__":
    raise SystemExit(main())
