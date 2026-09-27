#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 ROS 2 图像话题抓帧存 PNG。用途：把标定板拍下来，用 calibrate_mono.py
离线验证 --size（内角点数）填对了没有，免得在标定 GUI 里反复试。

  python3 snap.py --topic /image_raw --out ~/board.png
  python3 snap.py --topic /image_raw --count 5 --out ~/board --interval 1.0

存好之后验证尺寸（--detect-only 只检测、不标定、不写文件）：
  python3 calibrate_mono.py --images ~/board.png --size 11x8 --detect-only
  看到 ✔ ... 说明尺寸对了；看到 ✘ 就换成 8x11 或重新数内角点
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ros_img import to_bgr
from sensor_msgs.msg import Image


class Snap(Node):
    def __init__(self, args):
        super().__init__("snap")
        self.args = args
        self.saved = 0
        self.last_save = 0.0
        self.sub = self.create_subscription(
            Image, args.topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info("等待话题 %s 的图像 ..." % args.topic)

    def on_image(self, msg):
        now = time.time()
        if self.saved and now - self.last_save < self.args.interval:
            return
        try:
            img = to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e))
            raise SystemExit(1)

        if self.args.count > 1:
            base, ext = os.path.splitext(self.args.out)
            path = "%s_%02d%s" % (base, self.saved, ext or ".png")
        else:
            path = self.args.out if os.path.splitext(self.args.out)[1] else self.args.out + ".png"
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        import cv2
        cv2.imwrite(path, img)
        self.saved += 1
        self.last_save = now
        self.get_logger().info("已保存 %s  (%dx%d, %s)" % (path, msg.width, msg.height, msg.encoding))
        if self.saved >= self.args.count:
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/image_raw")
    ap.add_argument("--out", default=None,
                    help="输出路径；不给就存到 <脚本目录>/标定结果/<时间戳>_snap/board.png")
    ap.add_argument("--count", type=int, default=1)
    ap.add_argument("--interval", type=float, default=0.5, help="多张时的最小间隔（秒）")
    ap.add_argument("--timeout", type=float, default=20.0, help="等不到图就退出（秒）")
    args = ap.parse_args()
    if args.out:
        args.out = os.path.expanduser(args.out)
    else:
        root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "标定结果")
        session = os.path.join(root, time.strftime("%Y%m%d_%H%M%S_") + "snap")
        os.makedirs(session, exist_ok=True)
        try:
            latest = os.path.join(root, "latest_snap")
            if os.path.islink(latest):
                os.remove(latest)
            os.symlink(os.path.abspath(session), latest)
        except OSError:
            pass
        args.out = os.path.join(session, "board.png")

    rclpy.init()
    node = Snap(args)
    deadline = time.time() + args.timeout
    try:
        while rclpy.ok() and time.time() < deadline and node.saved < args.count:
            rclpy.spin_once(node, timeout_sec=0.2)
    except SystemExit as e:
        if e.code not in (0, None):
            return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.saved == 0:
        print("没收到任何图像：确认驱动在跑、话题名对（ros2 topic list）", file=sys.stderr)
        return 1
    print("\n下一步验证棋盘格尺寸（只检测、不标定）：")
    print("  python3 %s --images %s --size <宽x高内角点> --detect-only"
          % (os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibrate_mono.py"), args.out))
    print("  例如 12x9 个方格的板子 -> --size 11x8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
