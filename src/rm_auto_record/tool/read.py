#!/usr/bin/env python3

import argparse
import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage
import rosbag2_py

def read_messages(input_bag: str):
    """读取MCAP文件中的消息"""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"话题 {topic_name} 不在bag文件中")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp

def main():
    parser = argparse.ArgumentParser(description='从MCAP文件读取图像并转换为视频')
    parser.add_argument('input', help='输入的MCAP文件路径')
    parser.add_argument('--topic', help='要处理的图像话题名称', default='/image_raw/compressed')
    parser.add_argument('--output', help='输出视频文件路径', default='output.mp4')
    parser.add_argument('--fps', type=int, help='输出视频的帧率', default=30)

    args = parser.parse_args()

    # 创建视频写入器
    video_writer = None
    frame_count = 0

    print(f'正在处理MCAP文件: {args.input}')
    print(f'监听话题: {args.topic}')
    print(f'输出视频文件: {args.output}')

    try:
        for topic, msg, timestamp in read_messages(args.input):
            if topic == args.topic and isinstance(msg, CompressedImage):
                # 解码压缩图像
                np_arr = np.frombuffer(msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if video_writer is None:
                    height, width = frame.shape[:2]
                    video_writer = cv2.VideoWriter(
                        args.output,
                        cv2.VideoWriter_fourcc(*'mp4v'),
                        args.fps,
                        (width, height)
                    )

                video_writer.write(frame)
                frame_count += 1

                if frame_count % 100 == 0:
                    print(f'已处理 {frame_count} 帧')

    except KeyboardInterrupt:
        print('\n用户中断处理')
    finally:
        if video_writer is not None:
            video_writer.release()
            print(f'\n处理完成！共处理了 {frame_count} 帧')
            print(f'视频已保存到: {args.output}')

if __name__ == '__main__':
    main() 