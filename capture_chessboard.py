#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 V4L2/UVC 相机（USB 摄像头、UVC 模式的海康网络/工业相机）或 RTSP 视频流
（海康威视网络摄像机）采集棋盘格标定图。

键盘：
  空格  保存当前帧（检测到棋盘格才保存）
  a     切换自动采集：检测到棋盘格且相对上一张有明显位移就自动存
  u     切换去畸变预览（写入 YAML 后才有意义，可选）
  q/ESC 退出

示例：
  python3 capture_chessboard.py --device /dev/video0 --size 8x6 \
      --width 1440 --height 1080 --out ./images --auto

依赖：opencv-python、numpy；画面叠加的中文用 Pillow + 系统中文字体渲染，
      系统里没有中文字体时自动改用英文文案（不会出现问号方块）。
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np


def parse_size(text):
    cols, rows = text.lower().split("x")
    return int(cols), int(rows)


# --------------------------------------------------------------------------- #
# 叠加文字
# OpenCV 的 putText 只能画 ASCII，中文会被渲染成 "?????"（字体里没有 CJK 字形）。
# 这里优先用 PIL 画中文（Pillow 是图形界面已依赖的库），找不到中文字体就退回
# ASCII 提示——保证任何机器上都不会出现问号。
# --------------------------------------------------------------------------- #
_FONT_PATHS = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/System/Library/Fonts/PingFang.ttc",
    "C:/Windows/Fonts/msyh.ttc",
)
_font_cache = {}


def _cjk_font(size):
    """按需加载一个能画中文的字体；系统里找不到返回 None。"""
    if size not in _font_cache:
        font = None
        try:
            from PIL import ImageFont
            for path in _FONT_PATHS:
                if os.path.exists(path):
                    font = ImageFont.truetype(path, size)
                    break
        except Exception:
            font = None
        _font_cache[size] = font
    return _font_cache[size]


def draw_label(img, text, ascii_text, org=(12, 8), size=26, color_bgr=(0, 255, 0)):
    """在图像上画一行说明文字（可含中文），原地修改并返回 img。

    @param img        BGR 图像
    @param text       中文文案
    @param ascii_text 系统没有中文字体时改用的纯英文文案（必须可读，不能是问号）
    @param org        文字左上角坐标 (x, y)
    @param size       字号（像素）
    @param color_bgr  BGR 颜色
    @return           img
    @note  只转换文字所在的小矩形，避免整帧 BGR<->RGB 来回搬运。
    """
    font = _cjk_font(size)
    if font is None:                 # 没有中文字体：画英文文案，绝不出现问号
        cv2.putText(img, ascii_text, (org[0], org[1] + size),
                    cv2.FONT_HERSHEY_SIMPLEX, size / 30.0, color_bgr, 2)
        return img
    from PIL import Image, ImageDraw
    x, y = org
    box = font.getbbox(text)
    x0, y0 = max(0, x + box[0] - 2), max(0, y + box[1] - 2)
    x1 = min(img.shape[1], x + box[2] + 2)
    y1 = min(img.shape[0], y + box[3] + 2)
    if x1 <= x0 or y1 <= y0:
        return img
    patch = Image.fromarray(img[y0:y1, x0:x1][:, :, ::-1])          # BGR -> RGB
    ImageDraw.Draw(patch).text((x - x0, y - y0), text, font=font,
                               fill=(color_bgr[2], color_bgr[1], color_bgr[0]))
    img[y0:y1, x0:x1] = np.asarray(patch)[:, :, ::-1]               # RGB -> BGR
    return img


def open_camera(device, width, height, fps, fourcc, url=None):
    if url:
        # 海康威视网络摄像机：rtsp://user:pass@192.168.1.64:554/Streaming/Channels/101
        cap = cv2.VideoCapture(url)
    elif device.isdigit():
        cap = cv2.VideoCapture(int(device), cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit("打不开相机 %s。若是海康工业相机(MV 系列)，它不走 UVC，需用 MVS SDK 采图。" % (url or device))
    if not url:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        # 手动曝光更利于标定（避免自动曝光/自动增益抖动）
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = manual (V4L2)
    return cap


def detect(gray, pattern_size, use_sb=True):
    if use_sb and hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        ok, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=flags)
        if ok:
            return True, corners
        return False, None
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if ok:
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-4))
    return ok, corners


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0", help="V4L2 设备，如 /dev/video0 或 0")
    ap.add_argument("--url", default=None, help="RTSP 地址（海康网络摄像机），给了它则忽略 --device")
    ap.add_argument("--size", type=parse_size, default=(8, 6), help="内角点数 8x6")
    ap.add_argument("--width", type=int, default=1440)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--fourcc", default="MJPG", help="MJPG 可让 UVC 在高分辨率下跑满帧率")
    ap.add_argument("--out", default="./images")
    ap.add_argument("--prefix", default="img")
    ap.add_argument("--target", type=int, default=20, help="目标张数")
    ap.add_argument("--auto", action="store_true", help="启动即开启自动采集")
    ap.add_argument("--min-shift", type=float, default=25.0, help="自动采集时两张图的最小平均位移(px)")
    ap.add_argument("--no-sb", action="store_true", help="不用 findChessboardCornersSB")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    existing = [f for f in os.listdir(args.out) if f.startswith(args.prefix)]
    idx = len(existing)
    cap = open_camera(args.device, args.width, args.height, args.fps, args.fourcc, url=args.url)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("实际分辨率: %dx%d  输出目录: %s" % (w, h, os.path.abspath(args.out)))
    print("空格=保存  a=自动采集  q=退出   目标 %d 张" % args.target)

    auto = args.auto
    last_corners = None
    saved = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("读帧失败，退出")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = detect(gray, args.size, use_sb=not args.no_sb)
        view = frame.copy()
        shift = None
        if found:
            cv2.drawChessboardCorners(view, args.size, corners, found)
            if last_corners is not None and len(last_corners) == len(corners):
                shift = float(np.linalg.norm(
                    corners.reshape(-1, 2) - last_corners.reshape(-1, 2), axis=1).mean())
        status = "已存 %d/%d" % (saved, args.target)
        status_en = "saved %d/%d" % (saved, args.target)
        tip = "检测到标定板" if found else "未检测到"
        tip_en = "board OK" if found else "no board"
        if shift is not None:
            tip += "  位移 %.1f px" % shift
            tip_en += "  shift %.1f px" % shift
        draw_label(view,
                   "%s | %s | 自动:%s" % (status, tip, "开" if auto else "关"),
                   "%s | %s | auto:%s" % (status_en, tip_en, "on" if auto else "off"),
                   org=(12, 8), size=26,
                   color_bgr=(0, 255, 0) if found else (0, 0, 255))
        cv2.imshow("capture_chessboard", view)

        should_save = False
        if found and auto:
            if shift is None or shift >= args.min_shift or last_corners is None:
                should_save = True
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord(" "):
            should_save = found or False
            if not found:
                print("当前帧没检测到棋盘格，未保存")
        if key == ord("a"):
            auto = not auto
            print("自动采集:", "开" if auto else "关")

        if should_save:
            name = os.path.join(args.out, "%s_%03d.png" % (args.prefix, idx))
            cv2.imwrite(name, frame)
            idx += 1
            saved += 1
            last_corners = corners
            print("已保存 %s" % name)
            if saved >= args.target:
                print("已达到目标张数 %d，可以开始标定了。" % args.target)

    cap.release()
    cv2.destroyAllWindows()
    print("共保存 %d 张到 %s" % (saved, os.path.abspath(args.out)))
    print("下一步: python3 calibrate_mono.py --images %s --size %dx%d --square <实测方格边长(米)>"
          % (args.out, args.size[0], args.size[1]))


if __name__ == "__main__":
    main()
