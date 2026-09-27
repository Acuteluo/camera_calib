#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sensor_msgs/Image -> numpy 的转换（不依赖 rclpy，方便离线/自检复用）。"""

import numpy as np
import cv2


def to_rgb(msg):
    """返回 RGB 数组。rgb8 且行连续时是【零拷贝视图】，直接可用于 Tk 显示。
    实测：比先转 BGR 再切回来快 ~50 倍（0.11ms vs 5.35ms，1440x1080）。"""
    enc = msg.encoding.lower()
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc == "rgb8":
        if step == w * 3:
            return buf.reshape(h, w, 3)
        return buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3).copy()
    if enc == "bgr8":
        img = buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    if enc in ("rgba8", "bgra8"):
        img = buf.reshape(h, step)[:, : w * 4].reshape(h, w, 4)
        code = cv2.COLOR_RGBA2RGB if enc == "rgba8" else cv2.COLOR_BGRA2RGB
        return cv2.cvtColor(img, code)
    if enc in ("mono8", "8uc1"):
        g = buf.reshape(h, step)[:, :w]
        return cv2.cvtColor(g, cv2.COLOR_GRAY2RGB)
    return cv2.cvtColor(to_bgr(msg), cv2.COLOR_BGR2RGB)


def to_bgr(msg):
    """把 sensor_msgs/Image 转成 numpy 数组（返回 BGR 或灰度）。"""
    enc = msg.encoding.lower()
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
        return img[:, :, ::-1].copy() if enc == "rgb8" else img.copy()
    if enc in ("rgba8", "bgra8"):
        img = buf.reshape(h, step)[:, : w * 4].reshape(h, w, 4)
        return (img[:, :, 2::-1] if enc == "rgba8" else img[:, :, :3]).copy()
    if enc in ("mono8", "8uc1"):
        return buf.reshape(h, step)[:, :w].copy()
    if enc in ("mono16", "16uc1"):
        img = buf.reshape(h, step).view(np.uint16)[:, :w]
        return (img >> 4).astype(np.uint8)
    if enc in ("yuv422", "yuyv", "yuv422_yuy2", "uyvy", "yuv422_yuy2"):
        return buf.reshape(h, step)[:, : w * 2].reshape(h, w, 2)[:, :, 0].copy()
    if enc == "bayer_rggb8":
        # 贝叶斯不做去马赛克，抽偶数行偶数列当灰度图（标定只关心黑白格子）
        return buf.reshape(h, step)[:, :w][0::2, 0::2].copy()
    raise ValueError("暂不支持的编码 %s（请把驱动的像素格式改成 rgb8 或 mono8）" % msg.encoding)
