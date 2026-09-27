#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
用标定好的 ROS camera_info YAML 去畸变：验证标定结果好不好用。

  # 单张图，输出对比图
  python3 undistort_with_yaml.py --yaml hik_mv.yaml --image test.png --output check.jpg
  # 实时预览（xorg 桌面下）
  python3 undistort_with_yaml.py --yaml hik_mv.yaml --device /dev/video0
  # 海康网络摄像机 RTSP
  python3 undistort_with_yaml.py --yaml hik_mv.yaml --device rtsp://admin:pass@192.168.1.64:554/Streaming/Channels/101

键盘：s 保存当前帧  q/ESC 退出
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml


def load_camera_info(path):
    with open(path) as f:
        data = yaml.safe_load(f)

    def mat(key, rows, cols):
        d = data[key]
        return np.array(d["data"], dtype=np.float64).reshape(rows, cols)

    info = {
        "width": data["image_width"],
        "height": data["image_height"],
        "name": data.get("camera_name", "camera"),
        "K": mat("camera_matrix", 3, 3),
        "D": mat("distortion_coefficients", 1, data["distortion_coefficients"]["cols"]).ravel(),
        "R": mat("rectification_matrix", 3, 3),
        "P": mat("projection_matrix", 3, 4),
        "model": data.get("distortion_model", "plumb_bob"),
    }
    return info


def show_summary(info):
    K, D = info["K"], info["D"]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    hfov = 2 * np.degrees(np.arctan(info["width"] / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(info["height"] / (2 * fy)))
    print("相机: %s  分辨率 %dx%d  模型 %s" % (info["name"], info["width"], info["height"], info["model"]))
    print("fx=%.3f fy=%.3f cx=%.3f cy=%.3f" % (fx, fy, cx, cy))
    print("畸变: %s" % np.array2string(D, precision=6, separator=", "))
    print("视场角: 水平 %.2f°  垂直 %.2f°" % (hfov, vfov))


def make_maps(info, use_k=False):
    K, D = info["K"], info["D"]
    size = (info["width"], info["height"])
    newK = K if use_k else info["P"][:3, :3]
    if info["model"] == "equidistant" and hasattr(cv2, "fisheye"):
        mapx, mapy = cv2.fisheye.initUndistortRectifyMap(
            K, D.reshape(4, 1), info["R"], newK, size, cv2.CV_32FC1)
    else:
        mapx, mapy = cv2.initUndistortRectifyMap(K, D, info["R"], newK, size, cv2.CV_32FC1)
    return mapx, mapy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", required=True)
    ap.add_argument("--image")
    ap.add_argument("--device", default=None)
    ap.add_argument("--output", default="undistorted.jpg")
    ap.add_argument("--use-k", action="store_true",
                    help="用 K 而不是 YAML 里的 P 作为新内参（保留全部视野，边缘会有黑边）")
    args = ap.parse_args()
    if not args.image and not args.device:
        sys.exit("请给出 --image 或 --device")

    info = load_camera_info(args.yaml)
    show_summary(info)
    mapx, mapy = make_maps(info, use_k=args.use_k)

    if args.image:
        img = cv2.imread(args.image, cv2.IMREAD_COLOR)
        if img is None:
            sys.exit("读不到图片 %s" % args.image)
        if (img.shape[1], img.shape[0]) != (info["width"], info["height"]):
            print("[警告] 图片分辨率 %dx%d 与标定分辨率 %dx%d 不一致，结果不可信"
                  % (img.shape[1], img.shape[0], info["width"], info["height"]))
        und = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
        out = np.hstack([img, und])
        cv2.imwrite(args.output, out)
        print("原图 | 去畸变 对比已保存: %s" % os.path.abspath(args.output))
        return

    if args.device.startswith(("rtsp://", "http://", "https://")):
        cap = cv2.VideoCapture(args.device)
    else:
        dev = int(args.device) if args.device.isdigit() else args.device
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit("打不开相机 %s" % args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, info["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, info["height"])
    print("按 s 保存，q 退出")
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        und = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        cv2.imshow("raw | undistorted", np.hstack([frame, und]))
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            name = "undistorted_%03d.jpg" % n
            cv2.imwrite(name, und)
            print("已保存", name)
            n += 1
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
