#!/usr/bin/env python3
"""发布合成棋盘格 ROS 图像，无相机验证 GUI 全流程；默认独立话题。"""
import argparse
from pathlib import Path
import tempfile
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from selftest_synthetic import make_dataset


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/calib_demo/image_raw")
    ap.add_argument("--duration", type=float, default=90)
    ap.add_argument("--ready-file", type=Path)
    args = ap.parse_args()
    cv2.setNumThreads(2)
    with tempfile.TemporaryDirectory(prefix="calib_demo_") as tmp:
        make_dataset(tmp)
        frames = [cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
                  for path in sorted(Path(tmp).glob("*.png"))]
    rclpy.init()
    node = Node("calib_synthetic_camera")
    pub = node.create_publisher(Image, args.topic, qos_profile_sensor_data)
    started = time.monotonic()
    if args.ready_file:
        args.ready_file.write_text(args.topic + "\n")
    print("合成相机就绪：%s（8x6 内角点，25mm；测试图像）" % args.topic, flush=True)
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            gray = frames[int((time.monotonic() - started) / 1.0) % len(frames)]
            msg = Image()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "synthetic_camera"
            msg.height, msg.width = gray.shape
            msg.encoding = "mono8"
            msg.step = msg.width
            msg.data = gray.tobytes()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
