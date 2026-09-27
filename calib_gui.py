#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图形化相机标定工具（tkinter）

一次标定 = 启动相机包 → 摆板采集 → 标定 → 检验 → 输出。
所有操作都在窗口里点，不需要敲命令。

界面构成
--------
  顶部：驱动选择、启动/停止驱动、图像话题、实时状态
  左区：实时画面（检测到标定板会画彩色角点；可切换显示去畸变结果）
  右区：按钮 —— 开始/暂停采集、采集一张、标定、检验、输出到结果目录、写入驱动、退出
  底部：滑条 —— 曝光、增益、棋盘格宽、棋盘格高、方格边长(mm) + 应用尺寸并重新开始
  日志：驱动输出与操作记录

用法
----
  ./run_gui.sh                          # 推荐（会自动 source ROS 环境）
  ./run_gui.sh --driver mv --size 11x8 --square 0.025

  # 无相机的自检（用合成图跑通检测/标定/检验/输出全链路）
  python3 calib_gui.py --self-test
  # 在真实显示上自动演示（配合假相机话题）
  python3 calib_gui.py --demo --size 8x6 --square 0.025
"""

import argparse
import json
import os
import queue
import re
import subprocess
import sys
import threading
import time

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ace_json  # noqa: E402
from calibrate_mono import (calibrate, calibration_flags, detect_corners,  # noqa: E402
                            matching_points, ros_yaml)
from ros_img import to_rgb  # noqa: E402

RESULTS_ROOT = os.path.abspath(os.path.expanduser(
    os.environ.get("CALIB_RESULTS_DIR") or os.path.join(HERE, "标定结果")))
# 中文字体：按优先级挑系统里实际装了的那个（不是所有机器都有 Noto，见 README 环境要求）
UI_FONT_CANDIDATES = ("Noto Sans CJK SC", "Source Han Sans SC", "WenQuanYi Zen Hei",
                      "WenQuanYi Micro Hei", "Droid Sans Fallback",
                      "Noto Sans CJK JP", "Microsoft YaHei", "PingFang SC", "Heiti SC")
MONO_FONT_CANDIDATES = ("Noto Sans Mono CJK SC", "WenQuanYi Zen Hei Mono",
                        "WenQuanYi Micro Hei Mono", "Noto Sans Mono CJK JP",
                        "Droid Sans Fallback")
# 下面两个由 setup_fonts() 按系统实际字体改写（必须在建控件之前调用）
UI_FONT = "Noto Sans CJK SC"
MONO_FONT = "Noto Sans Mono CJK SC"   # 关键：Text 控件默认的 TkFixedFont 是
                                      # DejaVu Sans Mono（无中文字形），中文只能靠
                                      # fallback 拼，字距会乱


def setup_style(root, ttk, ui_font, ui_size=12):
    """把 ttk 控件（按钮/下拉/输入框/勾选框/滑条）调大，默认太小不好点。"""
    try:
        st = ttk.Style(root)
        st.configure("TButton", font=(ui_font, ui_size), padding=(12, 6))
        st.configure("TCombobox", font=(ui_font, ui_size), padding=(6, 4))
        st.configure("TEntry", font=(ui_font, ui_size), padding=(5, 4))
        st.configure("TCheckbutton", font=(ui_font, ui_size + 1), padding=(4, 4))
        st.configure("TLabelframe.Label", font=(ui_font, ui_size))
        st.configure("Horizontal.TScale", sliderlength=28)
        st.configure("TScrollbar", arrowsize=16)
        root.option_add("*TCombobox*Listbox.font", (ui_font, ui_size))
    except Exception:
        pass


def _pick_font(families, candidates):
    """从候选里挑第一个系统装了的字体族；都没有返回 None。"""
    for name in candidates:
        if name in families:
            return name
    return None


def setup_fonts(root):
    """挑一个系统里存在的**中文字体**，把 Tk 各默认字体统一到它上面。

    必须在创建任何控件之前调用（会改写模块级 UI_FONT / MONO_FONT）。
    @return 实际使用的等宽字体名（日志/报告文本框用）；没有中文字体时返回 None
    """
    global UI_FONT, MONO_FONT
    import tkinter.font as tkfont
    try:
        fams = set(tkfont.families(root))
    except Exception:
        return None
    ui = _pick_font(fams, UI_FONT_CANDIDATES)
    mono = _pick_font(fams, MONO_FONT_CANDIDATES)
    if ui:
        UI_FONT = ui
    if mono:
        MONO_FONT = mono
    for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont",
                 "TkTooltipFont", "TkIconFont"):
        try:
            f = tkfont.nametofont(name)
            if ui:
                f.configure(family=ui, size=11)
        except Exception:
            pass
    try:
        f = tkfont.nametofont("TkFixedFont")
        f.configure(family=mono or ui or "TkFixedFont", size=11)
        return f.actual("family")
    except Exception:
        return mono or ui
def default_camera_root():
    """三个相机驱动的工作空间根目录（本工具的“驱动”功能需要它们）。

    优先级：环境变量 CAMERA_ROOT > 常见位置自动探测 > 家目录下的默认猜测。
    三个驱动各自的子目录名是上游仓库名（ros2-hik-camera-main 等），换位置时只要保证
    目录名不变、或直接设 CAMERA_ROOT 指到新的根目录即可。
    """
    env = os.environ.get("CAMERA_ROOT")
    if env:
        return os.path.abspath(os.path.expanduser(env))
    home = os.path.expanduser("~")
    for cand in (os.path.join(home, "下载", "camera"),
                 os.path.join(home, "Downloads", "camera"),
                 os.path.join(home, "camera"),
                 os.path.join(home, "camera_ws"),
                 os.path.join(home, "ws", "camera"),
                 "/opt/camera"):
        if os.path.isdir(cand):
            return cand
    return os.path.join(home, "下载", "camera")


def build_drivers(camera_root):
    """按驱动根目录生成三个驱动的元信息表。"""
    return {
        "hik": dict(
            label="海康 MV-CS016-10UC", pkg="hik_camera", exe="hik_camera_node",
            ws=os.path.join(camera_root, "ros2-hik-camera-main"),
            node="/hik_camera", exp="exposure_time", gain="gain", cam_name="narrow_stereo"),
        "galaxy": dict(
            label="大恒 Galaxy", pkg="galaxy_camera", exe="galaxy_camera_node",
            ws=os.path.join(camera_root, "rm_vision_ros2_galaxy_camera-master"),
            node="/galaxy_camera", exp="exposure_time", gain="gain", cam_name="narrow_stereo"),
        "mv": dict(
            label="迈德威视", pkg="mindvision_camera", exe="mindvision_camera_node",
            ws=os.path.join(camera_root, "ros2_mindvision_camera"),
            node="/mv_camera", exp="exposure_time", gain="analog_gain", cam_name="mv_camera"),
    }


CAMERA_ROOT = default_camera_root()
DRIVERS = build_drivers(CAMERA_ROOT)

TOPIC = "/image_raw"
MAX_SAMPLES = 300                # 样本张数上限（覆盖度还能涨就该让用户继续采）
SAMPLE_MEMORY_BUDGET = 1_500_000_000   # 样本内存预算 1.5GB（超了按分辨率自动收紧张数）
MAX_RENDER_PIXELS = 3_000_000     # 显示图最大像素数（全屏约 1828x1371；再大贴图开始卡）
MIN_SAMPLES = 8          # 少于这么多张不允许标定（硬下限）
PREVIEW_DETECT_MAX = 480 # 预览检测先缩到最长边不超过这个像素（实测最优：~28ms）
RECOMMEND_SAMPLES = 40   # 对齐 ROS：≥40 张或覆盖度四条全绿才算"够"
GOODENOUGH_SAMPLES = 40

# 引导步骤条
STEPS = [("driver", "1 启动驱动"), ("capture", "2 采集样本"),
         ("calib", "3 标定"), ("check", "4 检验"), ("save", "5 输出")]


# =========================================================================== #
# 核心逻辑（与界面解耦，可被 --self-test 直接调用）
# =========================================================================== #
def detect_in(gray, size, pattern="chessboard", use_sb=True):
    """检测角点，返回 (corners, 每个角点对应的物点, 角点数)。unit 网格。"""
    imgp, objp = detect_corners(gray, size, pattern, use_sb=use_sb)
    return imgp, objp, (0 if imgp is None else len(imgp))


COVER_RANGES = [0.7, 0.7, 0.4, 0.5]        # X / Y / 距离 / 角度 需要覆盖的范围（同 ROS）
COVER_LABELS = [("x", "X 左右"), ("y", "Y 上下"), ("size", "距离"), ("skew", "角度")]


def coverage_params(corners, image_size, pattern_size):
    """抄自 ROS cameracalibrator 的 get_parameters：把一帧的板子姿态压成 4 个 0~1 的数。
    X/Y = 板子中心能跑多远；size = 板子在画面里占多大；skew = 倾斜程度。"""
    import math
    w, h = image_size
    xdim, ydim = pattern_size
    c = np.asarray(corners, np.float64).reshape(-1, 2)
    if c.shape[0] != xdim * ydim:
        return None
    ul, ur, dr, dl = c[0], c[xdim - 1], c[-1], c[-xdim]
    a, b, cc = ur - ul, dr - ur, dl - dr
    pvec, qvec = b + cc, a + b
    area = abs(pvec[0] * qvec[1] - pvec[1] * qvec[0]) / 2.0
    border = math.sqrt(area) if area > 0 else 0.0
    p_x = min(1.0, max(0.0, (c[:, 0].mean() - border / 2) / max(1.0, w - border)))
    p_y = min(1.0, max(0.0, (c[:, 1].mean() - border / 2) / max(1.0, h - border)))

    def angle(A, B, C):
        ab, cb = A - B, C - B
        d = np.linalg.norm(ab) * np.linalg.norm(cb)
        return math.acos(max(-1.0, min(1.0, float(np.dot(ab, cb)) / d))) if d > 0 else math.pi / 2

    skew = min(1.0, 2.0 * abs(math.pi / 2 - angle(ul, ur, dr)))
    return [p_x, p_y, math.sqrt(max(0.0, area) / (w * h)), skew]


def coverage_progress(sample_params):
    """返回 [(key, 中文名, progress 0~1, lo, hi)]；规则同 ROS：
    尺寸和角度只奖励“更大”，不奖励更小（min 记 0）。"""
    if not sample_params:
        return [(k, lab, 0.0, 0.0, 0.0) for k, lab in COVER_LABELS]
    arr = np.asarray(sample_params, np.float64)
    lo, hi = arr.min(axis=0), arr.max(axis=0)
    lo[2] = 0.0
    lo[3] = 0.0
    return [(k, lab, float(min((hi[i] - lo[i]) / COVER_RANGES[i], 1.0)), float(lo[i]), float(hi[i]))
            for i, (k, lab) in enumerate(COVER_LABELS)]


def _cover_color(prog):
    if prog >= 1.0:
        return "#3fa34d"      # 绿：够了
    if prog >= 0.5:
        return "#e0c31c"      # 黄
    return "#d9534f"          # 红：差得多


def fmt_param(key, v):
    """参数显示格式。方格边长(mm)必须支持小数——它直接影响焦距尺度，
    例如板子实测 24.7mm，四舍五入成 25 会带来 1.2% 的系统误差。"""
    if key == "gain":
        return "%.1f" % v
    if key == "sq":
        txt = "%.2f" % v
        return txt.rstrip("0").rstrip(".") if "." in txt else txt   # 25.00->25, 24.70->24.7
    return "%d" % round(v)          # 曝光/宽/高 仍是整数


def parse_param(key, txt):
    """把输入框文字解析成数值；返回 (值 或 None, 错误说明)。"""
    try:
        v = float(str(txt).strip())
    except ValueError:
        return None, "不是数字"
    if key == "sq":
        v = round(v, 2)
    elif key == "gain":
        v = round(v, 1)
    else:
        v = round(v)
    return v, None


def detect_preview(gray, size):
    """预览检测：findChessboardCornersSB + CALIB_CB_ACCURACY。
    实测（1440x1080 真实暗画面，pattern 8x6）：
      EXHAUSTIVE|ACCURACY 123.7ms / ACCURACY 45.1ms / classic 813ms（无板时）
    所以预览只用 ACCURACY，且外面还会先缩小到 480px 再调用。
    """
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(gray, size, flags=cv2.CALIB_CB_ACCURACY)
        if ok:
            return corners.reshape(-1, 1, 2).astype(np.float32)
        return None
    imgp, _objp = detect_corners(gray, size, "chessboard", use_sb=False)
    return imgp


class Sample:
    """一个采集样本：直接存灰度数组 + 角点。
    不再每张做 PNG 编码（1440x1080 要 30~60ms，会造成采集时卡顿），
    改为在“输出到结果目录”时一次性写盘。张数按内存上限控制。
    """

    __slots__ = ("arr", "corners", "shape")

    def __init__(self, gray, corners):
        self.arr = np.ascontiguousarray(gray)
        self.corners = corners
        self.shape = gray.shape  # (h, w)

    def gray(self):
        return self.arr


def calibrate_samples(samples, size, square, k=2):
    """用采集到的样本做单目标定，返回 dict（含 RMS、内参、每张误差、覆盖率）。

    性能：样本里已经存了预览阶段的角点（480px 检测，换算回全尺寸误差约 1~2px），
    这里直接在全尺寸灰度图上做 cornerSubPix 亚像素精修（每张几毫秒），
    不必再跑一遍完整检测（每张 100~800ms，300 张要算到 90 秒）。
    精修失败才退回完整检测。
    """
    objpoints, imgpoints = [], []
    board = matching_points(size, square, "chessboard")
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
    h_img, w_img = samples[0].shape[:2]
    for s in samples:
        gray = s.gray()
        imgp = None
        if s.corners is not None and len(s.corners) == size[0] * size[1]:
            try:
                c = np.asarray(s.corners, np.float32).reshape(-1, 1, 2)
                before = c.copy()      # cornerSubPix 修改入参并返回同一个对象 — 必须存副本
                imgp = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), crit)
                # 精修后验证：漂移超过 5px 或飞出图像边界 → 精修收敛到错误局部极值，
                # 退回完整 SB 重检。
                max_shift = float(np.linalg.norm(
                    (imgp - before).reshape(-1, 2), axis=1).max())
                in_bounds = (float(imgp[:, :, 0].min()) >= -1 and
                             float(imgp[:, :, 0].max()) <= w_img + 1 and
                             float(imgp[:, :, 1].min()) >= -1 and
                             float(imgp[:, :, 1].max()) <= h_img + 1)
                if max_shift > 5.0 or not in_bounds:
                    imgp = None   # 精修不可信，退回完整检测
            except Exception:
                imgp = None
        if imgp is None:
            imgp, _objp = detect_corners(gray, size, "chessboard", use_sb=True)
        if imgp is None:
            continue
        objpoints.append(board)
        imgpoints.append(imgp)
    if len(objpoints) < 4:
        raise RuntimeError("可用样本只有 %d 张（少于 4 张无法标定）" % len(objpoints))

    image_size = (samples[0].shape[1], samples[0].shape[0])
    args = argparse.Namespace(k=k, zero_tangent_dist=False,
                              fix_principal_point=False, fix_aspect_ratio=False)
    flags = calibration_flags(args)
    rms, K, D, per_view = calibrate(objpoints, imgpoints, image_size, flags)

    # 离群视图剔除：运动模糊/半板/误检的坏帧会把整体 RMS 拉高。
    # 剔除每张误差 > max(0.5px, 3×中位数) 的视图后重标定一次；最多剔 30%，且只在 RMS 变好时采用。
    n_rejected = 0
    if len(objpoints) >= 8:
        med = float(np.median(per_view))
        thr = max(0.5, 3.0 * med)
        keep = [i for i, e in enumerate(per_view) if e <= thr]
        if len(keep) >= 8 and len(keep) < len(objpoints) and len(keep) >= 0.7 * len(objpoints):
            op2 = [objpoints[i] for i in keep]
            ip2 = [imgpoints[i] for i in keep]
            rms2, K2, D2, pv2 = calibrate(op2, ip2, image_size, flags)
            if rms2 < rms:
                rms, K, D, per_view = rms2, K2, D2, pv2
                n_rejected = len(objpoints) - len(keep)
                objpoints, imgpoints = op2, ip2

    all_pts = np.concatenate([p.reshape(-1, 2) for p in imgpoints], axis=0).astype(np.float32)
    coverage = cv2.contourArea(cv2.convexHull(all_pts)) / float(image_size[0] * image_size[1])
    # 角点覆盖范围 u[min,max] v[min,max]：写进 ACE 配置注释，口径与工具报告一致
    uv_range = (int(round(float(all_pts[:, 0].min()))), int(round(float(all_pts[:, 0].max()))),
                int(round(float(all_pts[:, 1].min()))), int(round(float(all_pts[:, 1].max()))))
    # 去畸变用的新内参与投影矩阵（alpha=0：只保留有效像素）
    ncm, _ = cv2.getOptimalNewCameraMatrix(K, D, image_size, 0.0)
    P = np.zeros((3, 4))
    P[:3, :3] = ncm

    # 去畸变后角点偏离直线的程度（= ROS 界面里那个 lin.）
    n = size[0]
    und = cv2.undistortPoints(np.concatenate(imgpoints, 0).astype(np.float64), K, D)
    lin = _linear_error(und.reshape(-1, n, 2))
    return dict(rms=rms, K=K, D=D, P=P, per_view=per_view, image_size=image_size,
                coverage=coverage, linear=lin, n_views=len(objpoints),
                n_rejected=n_rejected, uv_range=uv_range)


def _linear_error(pts):
    """角点阵每行/每列偏离首尾连线的最大距离（像素）。"""
    devs = []
    for line in list(pts) + list(np.transpose(pts, (1, 0, 2))):
        p0, p1 = line[0], line[-1]
        d = p1 - p0
        nrm = np.hypot(*d)
        if nrm < 1e-9:
            continue
        n = np.array([-d[1], d[0]]) / nrm
        devs.append(np.abs((line - p0) @ n).max())
    return float(np.mean(devs)) if devs else float("nan")


def verdict_lines(res, square):
    K, D = res["K"], res["D"]
    rms, cov, lin = res["rms"], res["coverage"], res["linear"]
    ratio = K[0, 0] / K[1, 1] if K[1, 1] else float("nan")
    out = []
    out.append(("RMS 重投影误差  %.4f px" % rms,
                "优秀" if rms < 0.3 else ("合格" if rms < 0.5 else "不合格")))
    out.append(("角点覆盖画面    %.1f %%" % (cov * 100),
                "优秀" if cov > 0.6 else ("合格" if cov > 0.4 else "偏低")))
    out.append(("fx / fy 比值     %.4f" % ratio,
                "正常" if 0.99 <= ratio <= 1.01 else "异常"))
    out.append(("去畸变直线误差  %.3f px" % lin,
                "好" if lin < 1.0 else "偏差大"))
    out.append(("有效视图        %d 张" % res["n_views"],
                "够" if res["n_views"] >= GOODENOUGH_SAMPLES else "偏少（ROS 标准 ≥%d），建议再拍几张" % GOODENOUGH_SAMPLES))
    return out


def build_report(driver, res, square, size, topic):
    K, D = res["K"], res["D"]
    lines = []
    lines.append("相机标定检验报告")
    lines.append("=" * 46)
    lines.append("时间       : %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    lines.append("驱动       : %s" % DRIVERS[driver]["label"])
    lines.append("图像话题   : %s" % topic)
    lines.append("分辨率     : %dx%d" % res["image_size"])
    lines.append("棋盘格     : %d x %d 内角点，方格 %.2f mm" % (size[0], size[1], square * 1000))
    lines.append("")
    lines.append("-- 结论 --")
    for name, judge in verdict_lines(res, square):
        lines.append("  %-24s %s" % (name, judge))
    lines.append("")
    lines.append("-- 内参 --")
    lines.append("  fx = %.5f    fy = %.5f" % (K[0, 0], K[1, 1]))
    lines.append("  cx = %.5f    cy = %.5f" % (K[0, 2], K[1, 2]))
    lines.append("  畸变 (%s): %s" % ("plumb_bob" if D.size <= 5 else "rational_polynomial",
                                      ", ".join("%.6f" % x for x in D.ravel())))
    lines.append("")
    lines.append("-- 每张图的误差（从差到好）--")
    order = np.argsort(res["per_view"])[::-1]
    for i in order[:8]:
        lines.append("  #%02d  %.4f px" % (i + 1, res["per_view"][i]))
    if len(order) > 8:
        lines.append("  ... 其余 %d 张平均 %.4f px" % (
            len(order) - 8, float(np.mean([res["per_view"][i] for i in order[8:]]))))
    return "\n".join(lines)


def new_session(label):
    os.makedirs(RESULTS_ROOT, exist_ok=True)
    d = os.path.join(RESULTS_ROOT, time.strftime("%Y%m%d_%H%M%S_") + label)
    os.makedirs(d, exist_ok=True)
    latest = os.path.join(RESULTS_ROOT, "latest")
    try:
        if os.path.islink(latest):
            os.remove(latest)
        os.symlink(os.path.abspath(d), latest)
    except OSError:
        pass
    return d


def save_outputs(driver, res, samples, size, square, session=None, camera_name=None,
                 ace_template=None):
    """把内参/报告/采集图/去畸变对比写到时间戳目录，返回目录路径。

    ace_template 给出且文件存在时，额外写一份 ACE27 相机配置 JSON（JSONC）：与模板同构，
    只有 image_width/height、camera_matrix、distortion_model、distortion_coefficients、
    projection_matrix 这些内参字段不同；文件名与模板同名（如 uav.json），可直接拷回
    ACE27/config/cameras/。
    """
    import cv2
    session = session or new_session("%s_%dx%d" % (driver, size[0], size[1]))
    os.makedirs(session, exist_ok=True)
    K, D = res["K"], res["D"]
    R = np.eye(3)
    if res.get("P") is not None:              # 标定时已算过，直接复用
        P = res["P"]
        ncm = P[:3, :3]
    else:
        ncm, _ = cv2.getOptimalNewCameraMatrix(K, D, res["image_size"], 0.0)
        P = np.zeros((3, 4))
        P[:3, :3] = ncm
    cam = camera_name or DRIVERS[driver]["cam_name"]

    with open(os.path.join(session, "camera_info.yaml"), "w") as f:
        f.write(ros_yaml(cam, K, D, R, P, res["image_size"]))
    report = build_report(driver, res, square, size, TOPIC)
    with open(os.path.join(session, "检验报告.txt"), "w") as f:
        f.write(report + "\n")

    frames = os.path.join(session, "frames")
    os.makedirs(frames, exist_ok=True)
    for i, s in enumerate(samples):
        cv2.imwrite(os.path.join(frames, "left-%04d.png" % i), s.gray(),
                    [cv2.IMWRITE_PNG_COMPRESSION, 3])

    if samples:
        gray = samples[0].gray()
        mapx, mapy = cv2.initUndistortRectifyMap(K, D, R, ncm, res["image_size"], cv2.CV_32FC1)
        und = cv2.remap(gray, mapx, mapy, cv2.INTER_LINEAR)
        cv2.imwrite(os.path.join(session, "去畸变对比.jpg"),
                    np.hstack([cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
                               cv2.cvtColor(und, cv2.COLOR_GRAY2BGR)]))

    # ACE27 相机配置（可选）：以模板做文本替换，只动内参相关字段
    ace_name = "-"
    if ace_template and os.path.isfile(ace_template):
        try:
            model = "plumb_bob" if D.size <= 5 else "rational_polynomial"
            header = ace_json.build_header(
                when=time.strftime("%Y-%m-%d %H:%M:%S"),
                driver=DRIVERS[driver]["label"], pattern=size,
                square_mm=square * 1000.0, size=res["image_size"],
                n_views=res["n_views"], rms=res["rms"],
                uv=res.get("uv_range"), session=ace_json.rel_to_home(session))
            text = ace_json.build(ace_template, K, D, P, res["image_size"], model, header)
            out = os.path.join(session, ace_json.default_output_name(ace_template))
            with open(out, "w", encoding="utf-8") as f:
                f.write(text)
            ace_name = os.path.basename(out)
        except Exception as e:                                        # noqa: BLE001
            ace_name = "生成失败(%s)" % e

    with open(os.path.join(session, "说明.txt"), "w") as f:
        f.write("时间: %s\n驱动: %s\n棋盘格: %dx%d 内角点, 方格 %.1f mm\n"
                "分辨率: %dx%d\n有效视图: %d\nRMS: %.4f px\n覆盖: %.1f%%\nACE 配置: %s\n"
                % (time.strftime("%Y-%m-%d %H:%M:%S"), DRIVERS[driver]["label"],
                   size[0], size[1], square * 1000, res["image_size"][0], res["image_size"][1],
                   res["n_views"], res["rms"], res["coverage"] * 100, ace_name))
    return session


# =========================================================================== #
# ROS 采集线程
# =========================================================================== #
class CameraThread(threading.Thread):
    """订阅图像话题；预处理（缩放/去畸变/画角点）都在本线程做，UI 线程只贴图。
    检测限速见 state["interval"]（默认 0.08s ≈ 12Hz）。rclpy 由 main() 统一 init。"""

    def __init__(self, topic, state, lock):
        super().__init__(daemon=True)
        self.topic = topic
        self.state = state
        self.lock = lock
        self.node = None
        self._sub = None
        self._stop = False
        self._last_detect = 0.0
        self._fail_streak = 0
        self._last_ok_detect = 0.0

    def set_topic(self, topic):
        self.topic = topic
        if self.node is not None:
            try:
                self.node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = self._make_sub()

    def _make_sub(self):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        return self.node.create_subscription(Image, self.topic, self.on_image,
                                            qos_profile_sensor_data)

    def on_image(self, msg):
        # 性能要点（实测，1440x1080 真实画面）：
        #   cv2.findChessboardCorners 在“画面里没有板”时要 813ms（暗光噪声下候选爆炸），
        #   cv2.findChessboardCornersSB 只用 51ms；有板时 classic 2.6ms / SB 57ms。
        #   预览要“两种情况都有界”，所以预览用 SB，并且先缩到 <=720p 再检测。
        #   真正标定时由 calibrate_samples 用高精度 SB(EXHAUSTIVE|ACCURACY) 全尺寸重检。
        now = time.time()
        if os.environ.get("CALIB_GUI_DEBUG"):
            self._img_in = getattr(self, "_img_in", 0) + 1
        with self.lock:
            interval = self.state["interval"]
        if now - self._last_detect < interval:
            return
        self._last_detect = now
        if os.environ.get("CALIB_GUI_DEBUG"):
            self._img_ok = getattr(self, "_img_ok", 0) + 1
        try:
            rgb_full = to_rgb(msg)          # rgb8 时零拷贝，Tk 显示正好要 RGB
        except Exception as e:  # noqa: BLE001
            with self.lock:
                self.state["decode_err"] = str(e)
            return
        with self.lock:
            self.state["decode_err"] = None
            size = self.state["size"]
            want = self.state["want_size"]
            undist = self.state["undist"]
            maps = self.state["maps"]
        h, w = rgb_full.shape[:2]
        gray_full = rgb_full if rgb_full.ndim == 2 else cv2.cvtColor(rgb_full, cv2.COLOR_RGB2GRAY)

        # --- 1) 预览检测：常规缩到 <=480px（~28ms）；连续检不到就用原分辨率兜底试一次 ---
        k = min(1.0, PREVIEW_DETECT_MAX / float(max(w, h)))
        if k < 1.0 and self._fail_streak < 8:
            det_img = cv2.resize(gray_full, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
            det_k = k
        else:
            det_img, det_k = gray_full, 1.0     # 兜底：原尺寸找一次（板子小/远时更稳）
            self._fail_streak = 0
        try:
            imgp = detect_preview(det_img, (int(size[0]), int(size[1])))
        except Exception:
            imgp = None
        if imgp is not None:
            if det_k < 1.0:
                imgp = (np.asarray(imgp, np.float32) / det_k)   # 换回全尺寸坐标
            self._fail_streak = 0
            self._last_ok_detect = now
            if os.environ.get("CALIB_GUI_DEBUG"):
                self._img_det = getattr(self, "_img_det", 0) + 1
        else:
            self._fail_streak += 1
        n = 0 if imgp is None else len(imgp)

        # --- 2) 显示图：缩放/去畸变/画角点都在本线程做完，UI 线程只负责贴图 ---
        view = rgb_full
        if undist and maps is not None and (w, h) == maps[2]:
            view = cv2.remap(view, maps[0], maps[1], cv2.INTER_LINEAR)
        dw, dh = w, h
        if want:
            sc = min(want[0] / float(w), want[1] / float(h))        # 允许放大填满
            dw, dh = max(1, int(w * sc)), max(1, int(h * sc))
        if (dw, dh) != (w, h):
            view = cv2.resize(view, (dw, dh), interpolation=cv2.INTER_AREA)
        elif (imgp is not None or undist):
            view = view.copy()                  # 零拷贝视图不能直接画
        if imgp is not None:
            sc2 = dw / float(w)
            pts = (np.asarray(imgp, np.float32).reshape(-1, 2) * sc2).reshape(-1, 1, 2)
            cv2.drawChessboardCorners(view, (int(size[0]), int(size[1])), pts, True)

        with self.lock:
            self.state["native"] = (w, h)
            self.state["disp_rgb"] = view                 # 已是 RGB，UI 直接贴图
            self.state["bgr"] = view                      # 兼容“当前帧是否存在”的判断
            self.state["corners"] = imgp
            self.state["ncorner"] = n
            self.state["gray"] = gray_full
            self.state["stamp"] = now

    def run(self):
        import rclpy
        from rclpy.node import Node
        self.node = Node("calib_gui")
        self._sub = self._make_sub()
        try:
            while not self._stop:
                try:
                    rclpy.spin_once(self.node, timeout_sec=0.2)
                except Exception:
                    break          # 上下文被关掉时（rclpy.shutdown）正常退出
        finally:
            try:
                self.node.destroy_node()
            except Exception:
                pass

    def stop(self):
        self._stop = True


def _gray(bgr):
    import cv2
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


# =========================================================================== #
# 界面
# =========================================================================== #
def main():
    global DRIVERS, RESULTS_ROOT        # 允许命令行覆盖驱动根目录与结果目录
    ap = argparse.ArgumentParser()
    ap.add_argument("--driver", default="hik", choices=("hik", "galaxy", "mv"))
    ap.add_argument("--size", default="11x8", help="内角点数，如 11x8（12x9 个方格 -> 11x8）")
    ap.add_argument("--square", type=float, default=0.025, help="方格边长，米（默认 25mm）")
    ap.add_argument("--topic", default=TOPIC)
    ap.add_argument("--camera-root", default=None,
                    help="三个相机驱动的工作空间根目录（默认自动探测，或设环境变量 CAMERA_ROOT）")
    ap.add_argument("--driver-defaults", action="store_true",
                    help="启动时从驱动的 camera_params.yaml 读曝光/增益覆盖默认值（默认不读）")
    ap.add_argument("--results-dir", default=None,
                    help="标定结果归档目录（默认 <脚本目录>/标定结果，或设 CALIB_RESULTS_DIR）")
    ap.add_argument("--ace-template", default=ace_json.DEFAULT_TEMPLATE,
                    help="ACE27 相机配置模板（JSONC）；输出结果时据此生成同名 JSON")
    ap.add_argument("--self-test", action="store_true", help="无相机，用合成图跑通全链路")
    ap.add_argument("--demo", action="store_true", help="自动演示：自动采集→标定→检验→输出")
    ap.add_argument("--demo-calibrate-after", type=float, default=12.0)
    ap.add_argument("--demo-exit-after", type=float, default=0.0,
                    help="演示时多久后自动退出（0=不退出）")
    ap.add_argument("--geometry", default=None, help="窗口位置，如 0+0（测试用）")
    ap.add_argument("--demo-switch", metavar="驱动", default=None,
                    help="自测：启动后自动切到指定驱动并启动它，验证切换后环境正确（hik/galaxy/mv）")
    args = ap.parse_args()

    size = tuple(int(x) for x in args.size.lower().split("x"))

    # 换机器不用改源码：命令行 > 环境变量 > 自动探测（见 default_camera_root）
    if args.camera_root:
        CAMERA_ROOT = os.path.abspath(os.path.expanduser(args.camera_root))
        DRIVERS = build_drivers(CAMERA_ROOT)
    if args.results_dir:
        RESULTS_ROOT = os.path.abspath(os.path.expanduser(args.results_dir))

    if args.self_test:
        return self_test(size, args.square)

    import tkinter as tk
    from tkinter import ttk
    from PIL import Image, ImageTk

    class App:
        def __init__(self, root):
            self.root = root
            self.lock = threading.Lock()
            self.state = dict(bgr=None, gray=None, stamp=0.0, corners=None,
                              ncorner=0, size=size, decode_err=None, disp_rgb=None,
                              want_size=None, undist=False, maps=None, interval=0.08)
            self.driver = args.driver
            self.topic = args.topic
            self.ace_template = args.ace_template
            self.samples = []
            self.sample_params = []
            self.res = None
            self.session = None
            self.driver_proc = None
            self.driver_env = None
            self.logq = queue.Queue()
            self._photo = None
            self._last_save = 0.0
            self._last_corners = None
            self._last_param_sent = {}
            self._rate = []
            self.auto_collect = tk.BooleanVar(value=bool(args.demo))
            self.show_undist = tk.BooleanVar(value=False)
            self.min_shift = 18.0
            self.checked = False
            self.saved = False
            self._last_hint = None
            self._last_corner_ts = 0.0
            self._last_stamp = None
            self._t0 = time.time()
            self.max_samples = MAX_SAMPLES
            self.sample_bytes = 0
            self.driver_log = None
            self._cap_warned = False
            self._pub_t = 0.0
            self._pub_warn = None
            self._photo = None
            self._img_item = None
            self._want_size = None
            self._native_size = None

            root.title("相机标定工具 - %s%s" % (DRIVERS[args.driver]["label"],
                                               os.environ.get("CALIB_GUI_TITLE", "")))
            root.minsize(760, 520)      # 否则窗口会被控件"自然尺寸"撑到屏幕外
            root.protocol("WM_DELETE_WINDOW", self.quit)
            self._geom = args.geometry

            self._build_top(ttk)
            self._build_middle(ttk, tk)
            self._build_bottom(ttk, tk)

            self._load_driver_defaults()
            self.cam = CameraThread(self.topic, self.state, self.lock)
            self.cam.start()
            self.log("就绪。驱动=%s 棋盘格=%dx%d 方格=%.1fmm 话题=%s"
                     % (DRIVERS[self.driver]["label"], size[0], size[1], args.square * 1000, self.topic))
            if getattr(args, "demo_switch", None):
                tgt = args.demo_switch
                self.log("自测：3 秒后自动切到【%s】并启动它" % DRIVERS[tgt]["label"])
                self.root.after(3000, lambda: self._demo_switch_to(tgt))
            if args.demo:
                self.log("演示模式：自动采集，%.0f 秒后自动标定+检验+输出" % args.demo_calibrate_after)
                root.after(int(args.demo_calibrate_after * 1000), self.demo_finish)
            if args.demo_exit_after:
                root.after(int(args.demo_exit_after * 1000), self.quit)
            self.tick()

        # ---------------- 界面构建 ----------------
        def _build_top(self, ttk):
            """顶部一行：驱动 + 启停 + 话题 + 步骤条，全部挤一行，省纵向空间。"""
            f = ttk.Frame(self.root, padding=(6, 2))
            self._top = f
            f.grid(row=0, column=0, sticky="ew")
            ttk.Label(f, text="驱动:", font=(UI_FONT, 12)).grid(row=0, column=0, sticky="w")
            self.drv_var = tk.StringVar(value=DRIVERS[self.driver]["label"])
            cb = ttk.Combobox(f, textvariable=self.drv_var, width=16, state="readonly",
                              values=[DRIVERS[k]["label"] for k in DRIVERS])
            cb.grid(row=0, column=1, padx=(2, 8))
            cb.bind("<<ComboboxSelected>>", self.on_driver_change)
            self.btn_start = ttk.Button(f, text="启动驱动", command=self.start_driver, width=10)
            self.btn_start.grid(row=0, column=2, padx=2)
            self.btn_stop = ttk.Button(f, text="停止驱动", command=self.stop_driver,
                                       width=10, state="disabled")
            self.btn_stop.grid(row=0, column=3, padx=2)
            ttk.Label(f, text="话题:", font=(UI_FONT, 12)).grid(row=0, column=4, padx=(10, 2))
            self.topic_var = tk.StringVar(value=self.topic)
            ttk.Entry(f, textvariable=self.topic_var, width=10).grid(row=0, column=5)
            ttk.Button(f, text="重订阅", command=self.resubscribe, width=8) \
                .grid(row=0, column=6, padx=2)
            f.columnconfigure(7, weight=1)          # 中间留白，把步骤条顶到最右
            self.step_labels = {}
            sg = ttk.Frame(f)
            sg.grid(row=0, column=8, padx=(14, 0), sticky="e")
            for i, (name, text) in enumerate(STEPS):
                if i:      # 步骤之间用箭头连起来
                    tk.Label(sg, text="→", font=(UI_FONT, 14, "bold"),
                             fg="#808080", bg="#dcdad5") \
                        .grid(row=0, column=i * 2 - 1, padx=1)
                lb = tk.Label(sg, text=text, width=9, padx=6, pady=4, borderwidth=1,
                              relief="solid", font=(UI_FONT, 11, "bold"))
                lb.grid(row=0, column=i * 2, padx=1)
                self.step_labels[name] = lb

            g = ttk.Frame(self.root, padding=(6, 0))
            g.grid(row=1, column=0, sticky="ew")
            self.status = tk.StringVar(value="等待图像 ...")
            ttk.Label(g, textvariable=self.status, font=(UI_FONT, 13, "bold")) \
                .grid(row=0, column=0, sticky="w")
            # 提示单独一行：和状态挤一行会把窗口顶宽
            self.hint = tk.StringVar(value="")
            self.hint_label = ttk.Label(g, textvariable=self.hint, foreground="#0645ad",
                                        font=(UI_FONT, 13, "bold"), wraplength=1900)
            self.hint_label.grid(row=1, column=0, sticky="w", pady=(2, 0))

        def _build_middle(self, ttk, tk):
            """图像（占满剩余空间）+ 右侧按钮。"""
            f = ttk.Frame(self.root, padding=(4, 1))
            self._mid = f
            f.grid(row=2, column=0, sticky="nsew")
            self.root.columnconfigure(0, weight=1)
            self.root.rowconfigure(2, weight=1, minsize=460)   # 图像行拿走所有多余高度
            f.columnconfigure(0, weight=1)
            f.rowconfigure(0, weight=1)

            # Canvas 显示图像：画布元素不会反过来撑大控件，窗口尺寸才由我们说了算
            self.img_canvas = tk.Canvas(f, width=480, height=360, background="#d9d9d9",
                                        highlightthickness=0)
            self.img_canvas.grid(row=0, column=0, sticky="nsew")
            self.img_canvas.bind("<Configure>", self._on_img_resize)

            p = ttk.Frame(f, padding=(8, 0))
            p.grid(row=0, column=1, sticky="nsew")
            p.rowconfigure(9, weight=1)          # 日志那行吃掉纵向剩余空间
            self.btns = {}
            spec = [("collect", "暂停采集" if args.demo else "开始采集", self.toggle_collect),
                    ("grab", "采集一张", self.grab_one),
                    ("calib", "标定", self.do_calibrate),
                    ("check", "检验报告", self.do_check),
                    ("save", "输出结果", self.do_save),
                    ("ace", "导出 ACE json", self.do_export_ace),
                    ("write", "写入驱动", self.do_write_driver),
                    ("apply", "应用尺寸", self.apply_size),
                    ("exit", "退出", self.quit)]
            for i, (key, text, cmd) in enumerate(spec):
                b = ttk.Button(p, text=text, command=cmd, width=15)
                b.grid(row=i // 2, column=i % 2, pady=3, padx=2, sticky="ew")
                self.btns[key] = b
            nrow = (len(spec) + 1) // 2
            # 自绘大号勾选框：方框 24x24、勾线 3px，比系统自带的小方块明显得多
            chkf = tk.Frame(p, bg="#dcdad5")
            chkf.grid(row=nrow, column=0, columnspan=2, pady=(8, 0), sticky="w")
            self.chk_canvas = tk.Canvas(chkf, width=24, height=24, bg="#dcdad5",
                                        highlightthickness=0, cursor="hand2")
            self.chk_canvas.pack(side="left")
            self.chk_canvas.create_rectangle(2, 2, 23, 23, outline="#444444",
                                             width=2, fill="#ffffff")
            self.chk_tick = self.chk_canvas.create_line(
                6, 13, 10, 18, 19, 6, width=3, fill="#1a7f37", state="hidden")
            chk_lb = tk.Label(chkf, text="显示去畸变", font=(UI_FONT, 13), bg="#dcdad5",
                              cursor="hand2")
            chk_lb.pack(side="left", padx=(8, 0))
            for w in (chkf, self.chk_canvas, chk_lb):
                w.bind("<Button-1>", lambda _e: self._toggle_undist())
            self.res_var = tk.StringVar(value="尚未标定")
            ttk.Label(p, textvariable=self.res_var, justify="left", foreground="#0a5",
                      font=(UI_FONT, 12, "bold")) \
                .grid(row=nrow + 1, column=0, columnspan=2, pady=(8, 4), sticky="w")

            # 日志放在右侧按钮下方并撑满：右侧不再留大片空白，日志也够大
            lf = ttk.LabelFrame(p, text="日志", padding=3)
            lf.grid(row=9, column=0, columnspan=2, sticky="nsew", pady=(4, 0))
            lf.columnconfigure(0, weight=1)
            lf.rowconfigure(0, weight=1)
            self.log_text = tk.Text(lf, height=16, width=46, wrap="char",
                                    font=(getattr(self, "mono_family", None) or "TkFixedFont", 12))
            self.log_text.grid(row=0, column=0, sticky="nsew")
            sb = ttk.Scrollbar(lf, command=self.log_text.yview)
            sb.grid(row=0, column=1, sticky="ns")
            self.log_text.configure(yscrollcommand=sb.set)

        def _build_bottom(self, ttk, tk):
            """覆盖度进度条（大）+ 参数行（紧凑）。"""
            cvf = ttk.LabelFrame(self.root, padding=(4, 2),
                                 text="覆盖度（四条都变绿最稳；按没绿的那条补姿态）")
            self._cov = cvf
            cvf.grid(row=3, column=0, sticky="ew", padx=4, pady=(1, 1))
            self.cov_bars = {}
            cvf.columnconfigure(tuple(range(len(COVER_LABELS))), weight=1)
            for i, (key, lab) in enumerate(COVER_LABELS):
                cell = tk.Frame(cvf)
                cell.grid(row=0, column=i, sticky="ew", padx=(6, 12))
                tk.Label(cell, text=lab, font=(UI_FONT, 12, "bold")) \
                    .pack(side="left", padx=(0, 6))
                pc = tk.Label(cell, text="  0%", anchor="w", width=5,
                              font=(UI_FONT, 12, "bold"))
                pc.pack(side="right", padx=(6, 0))
                # 请求宽度给小，靠 fill+expand 撑满该列：窗口再窄也不会挤出边界
                cv = tk.Canvas(cell, width=1, height=24, highlightthickness=1,
                               highlightbackground="#a0a0a0", bg="#f2f2f2")
                cv.pack(side="left", fill="x", expand=True)
                self.cov_bars[key] = (cv, pc)

            self._param = f = ttk.LabelFrame(
                self.root, padding=(4, 2),
                text="参数（拖滑条，或点输入框直接输数字后回车；曝光/增益立即下发到相机）")
            f.grid(row=4, column=0, sticky="ew", padx=4, pady=(0, 1))
            self.vars = {}
            self.entries = {}
            self.entry_vars = {}
            self.ranges = {}
            spec = [("exp", "曝光(us)", 100, 200000, 5000),
                    ("gain", "增益(dB)", 0, 30, 15),
                    ("sq", "方格(mm)", 1, 200, round(args.square * 1000.0, 2)),
                    ("sw", "宽(角点)", 3, 20, size[0]),
                    ("sh", "高(角点)", 3, 20, size[1])]
            f.columnconfigure(tuple(range(len(spec))), weight=1)   # 五组均匀铺开
            for i, (key, text, lo, hi, init) in enumerate(spec):
                self.ranges[key] = (lo, hi)
                # 一组 = 文字 + 滑条 + 输入框，装在自己的子框里，组间留出间距
                cell = ttk.Frame(f)
                cell.grid(row=0, column=i, sticky="ew", padx=(2, 14), pady=3)
                ttk.Label(cell, text=text, font=(UI_FONT, 12)) \
                    .pack(side="left", padx=(0, 8))
                v = tk.DoubleVar(value=init)
                self.vars[key] = v
                e = ttk.Entry(cell, textvariable=tk.StringVar(), width=8, justify="right")
                # 先把输入框放到最右，再让滑条吃掉中间的宽度
                s = ttk.Scale(cell, from_=lo, to=hi, variable=v, length=40,
                              command=lambda _=None, k=key: self.on_scale(k))
                s.pack(side="left", fill="x", expand=True)
                ev = tk.StringVar(value=self._fmt(key, init))
                self.entry_vars[key] = ev
                e.configure(textvariable=ev)
                e.pack(side="right", padx=(8, 0))
                e.bind("<Return>", lambda _e, k=key: self.apply_entry(k))
                e.bind("<FocusOut>", lambda _e, k=key: self.apply_entry(k))
                self.entries[key] = e
                if key in ("exp", "gain"):
                    s.bind("<ButtonRelease-1>", lambda _e, k=key: self.apply_exp_gain(k))

        def _fmt(self, key, v):
            return fmt_param(key, v)

        def on_scale(self, key):
            """滑条动了就回写输入框；但用户正在框里打字时不打断。"""
            try:
                if self.entries[key] is self.root.focus_get():
                    return
            except Exception:
                pass
            self.entry_vars[key].set(self._fmt(key, self.vars[key].get()))

        def apply_entry(self, key):
            """输入框回车/失焦：解析成数字、夹到范围内、同步滑条并下发。"""
            lo, hi = self.ranges[key]
            txt = self.entry_vars[key].get().strip()
            v, err = parse_param(key, txt)
            if err:
                self.entry_vars[key].set(self._fmt(key, self.vars[key].get()))
                self.log("输入不是数字（%s），已还原为 %s"
                         % (txt, self.entry_vars[key].get()))
                return
            v = max(lo, min(hi, v))
            if abs(v - float(txt)) > 1e-9:
                self.log("%s 超出范围，已夹到 %s（允许 %s ~ %s）"
                         % (key, self._fmt(key, v), self._fmt(key, lo), self._fmt(key, hi)))
            self.vars[key].set(v)
            self.entry_vars[key].set(self._fmt(key, v))
            if key in ("exp", "gain"):
                self.apply_exp_gain(key)
            elif key in ("sw", "sh", "sq"):
                self.log("提示：尺寸/方格改完要点右侧【应用尺寸并重新开始】才生效")

        def log(self, msg):
            line = "[%s] %s" % (time.strftime("%H:%M:%S"), msg)
            self.logq.put(line)
            if os.environ.get("CALIB_GUI_DEBUG"):
                print(line, flush=True)

        def on_driver_change(self, _evt=None):
            label = self.drv_var.get()
            for k, d in DRIVERS.items():
                if d["label"] == label:
                    self.driver = k
                    break
            # 切换驱动必须重建环境（各驱动是各自的工作空间），并停掉正在跑的旧驱动
            if self._driver_running():
                self.stop_driver()
                self.log("切换驱动：已停止上一个驱动")
            self.driver_env = None
            self._last_param_sent.clear()
            self._load_driver_defaults()
            self.root.title("相机标定工具 - %s" % DRIVERS[self.driver]["label"])
            self.log("已切到【%s】，点【启动驱动】开始" % DRIVERS[self.driver]["label"])

        def _load_driver_defaults(self):
            """从驱动的 camera_params.yaml 读曝光/增益覆盖默认值。

            默认**不读**：工具自带默认值（曝光 5000us、增益 15dB）更可预测；
            需要跟随驱动配置时加 --driver-defaults。
            """
            if not args.driver_defaults:
                return
            import re as _re
            d = DRIVERS[self.driver]
            path = os.path.join(d["ws"], "config", "camera_params.yaml")
            try:
                with open(path) as f:
                    txt = f.read()
            except OSError:
                return
            def grab(key):
                m = _re.search(r"^\s*%s:\s*([0-9.]+)" % _re.escape(key), txt, _re.M)
                return float(m.group(1)) if m else None
            exp = grab(d["exp"])
            gain = grab(d["gain"])
            if exp is not None:
                self.vars["exp"].set(exp)
                self.on_scale("exp")
            if gain is not None:
                self.vars["gain"].set(gain)
                self.on_scale("gain")
            if exp is not None or gain is not None:
                self.log("已载入该驱动配置里的默认值：曝光 %s us，增益 %s"
                         % (self._fmt("exp", self.vars["exp"].get()) if exp is not None else "-",
                            self._fmt("gain", self.vars["gain"].get()) if gain is not None else "-"))

        def _preflight_driver(self):
            """启动前的通用检查：返回 True 表示可以启动。"""
            d = DRIVERS[self.driver]
            if not os.path.isfile(os.path.join(d["ws"], "install", "setup.bash")):
                self.log("该驱动还没编译：%s/install 不存在，先在该目录执行 colcon build" % d["ws"])
                return False
            if self.driver == "galaxy":
                # 大恒 SDK 初始化时要写这个日志目录，不可写会报 GXInitApi failed
                logdir = "/var/log/Galaxy"
                if not os.access(logdir, os.W_OK):
                    self.log("大恒 SDK 需要可写的 %s，请先在终端执行：" % logdir)
                    self.log("    sudo mkdir -p /var/log/Galaxy && sudo chmod 777 /var/log/Galaxy")
                    return False
            exe = os.path.join(d["ws"], "install", d["pkg"], "lib", d["pkg"], d["exe"])
            if os.path.isfile(exe) and not os.access(exe, os.X_OK):
                try:
                    os.chmod(exe, 0o755)
                    self.log("已修复缺失的可执行权限: %s" % exe)
                except OSError as e:
                    self.log("可执行文件没有执行权限且改不动：%s（%s）" % (exe, e))
                    return False
            return True

        def resubscribe(self):
            self.topic = self.topic_var.get().strip() or TOPIC
            self.cam.set_topic(self.topic)
            with self.lock:
                self.state["bgr"] = None
                self.state["corners"] = None
            self.log("已切换订阅话题 %s" % self.topic)

        # ---------------- 驱动 ----------------
        def _driver_env(self):
            if self.driver_env is None:
                ws = DRIVERS[self.driver]["ws"]
                ros = os.environ.get("ROS_DISTRO")
                ros_setup = ("/opt/ros/%s/setup.bash" % ros
                             if ros and os.path.isfile("/opt/ros/%s/setup.bash" % ros)
                             else ("/opt/ros/humble/setup.bash"
                                   if os.path.isfile("/opt/ros/humble/setup.bash") else ""))
                code = ('%ssource "%s/install/setup.bash" >/dev/null 2>&1; '
                        'python3 -c "import os,json;print(json.dumps(dict(os.environ)))"'
                        % ('source "%s" >/dev/null 2>&1; ' % ros_setup if ros_setup else "", ws))
                out = subprocess.check_output(["bash", "-c", code], text=True)
                self.driver_env = json.loads(out)
            return self.driver_env

        def start_driver(self):
            if self.driver_proc and self.driver_proc.poll() is None:
                self.log("驱动已在运行")
                return
            d = DRIVERS[self.driver]
            if not self._preflight_driver():
                return
            try:
                env = self._driver_env()
            except Exception as e:
                self.log("准备驱动环境失败: %s" % e)
                return
            # 驱动日志单独放 <结果根>/驱动日志/ 下：**不建标定会话**。
            # 否则只是点一下【启动驱动】就会冒出一个空的"标定结果"目录，还会顶掉 latest。
            logdir = os.path.join(RESULTS_ROOT, "驱动日志")
            os.makedirs(logdir, exist_ok=True)
            logf = os.path.join(logdir, time.strftime("%Y%m%d_%H%M%S_driver.log"))
            self._logf = open(logf, "w")
            self.driver_log = logf
            self.log("启动 %s ...（日志 %s）" % (d["label"], logf))
            self.driver_proc = subprocess.Popen(
                ["ros2", "run", d["pkg"], d["exe"]], env=env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
            threading.Thread(target=self._pump_driver_log, daemon=True).start()
            self.btn_start.configure(state="disabled")
            self.btn_stop.configure(state="normal")
            self.root.after(2500, lambda: self.apply_exp_gain("exp", initial=True))

        def _pump_driver_log(self):
            p = self.driver_proc
            for line in p.stdout:
                self._logf.write(line)
                self._logf.flush()
                s = line.rstrip()
                if s:
                    self.logq.put("驱动| " + s[-160:])
            self.log("驱动已退出")

        def stop_driver(self):
            if self.driver_proc and self.driver_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.driver_proc.pid), 15)
                except Exception:
                    self.driver_proc.terminate()
                self.log("已停止驱动")
            self.btn_start.configure(state="normal")
            self.btn_stop.configure(state="disabled")

        def apply_exp_gain(self, key, initial=False):
            if not (self.driver_proc and self.driver_proc.poll() is None):
                if not initial:
                    self.log("驱动没在运行，曝光/增益未下发（可先点“启动驱动”）")
                return
            d = DRIVERS[self.driver]
            vals = {d["exp"]: int(round(self.vars["exp"].get())),
                    d["gain"]: float("%.1f" % self.vars["gain"].get())}
            for name, val in vals.items():
                if not initial and self._last_param_sent.get(name) == val:
                    continue
                self._last_param_sent[name] = val
                threading.Thread(target=self._set_param, args=(d["node"], name, val),
                                 daemon=True).start()

        def _set_param(self, node, name, val):
            try:
                r = subprocess.run(["ros2", "param", "set", node, name, str(val)],
                                   env=self._driver_env(), capture_output=True, text=True, timeout=10)
                msg = (r.stdout + r.stderr).strip().splitlines()
                self.log("参数 %s=%s -> %s" % (name, val, msg[-1] if msg else "无输出"))
            except Exception as e:
                self.log("设置 %s 失败: %s" % (name, e))

        # ---------------- 采集 ----------------
        def _toggle_undist(self):
            self.show_undist.set(not self.show_undist.get())
            try:
                self.chk_canvas.itemconfigure(
                    self.chk_tick, state="normal" if self.show_undist.get() else "hidden")
            except Exception:
                pass

        def toggle_collect(self):
            self.auto_collect.set(not self.auto_collect.get())
            self.btns["collect"].configure(text="暂停采集" if self.auto_collect.get() else "开始采集")
            self.log("自动采集: %s" % ("开" if self.auto_collect.get() else "关"))

        def grab_one(self):
            with self.lock:
                corners, gray = self.state["corners"], self.state["gray"]
            if corners is None or gray is None:
                self.log("这一帧没检测到标定板，没有采集")
                return
            if len(self.samples) >= self.max_samples:
                self.log("样本已满 %d 张（约 %d MB）：可先【标定】，或点【应用尺寸】清空重来"
                         % (self.max_samples, self.sample_bytes // (1024 * 1024)))
                return
            s_new = Sample(gray, corners)
            self.samples.append(s_new)
            self.sample_bytes += s_new.arr.nbytes
            self._record_cover(corners, gray.shape)
            self._last_corners = corners
            self._last_save = time.time()
            self.log("已采集第 %d 张（%d 角点）" % (len(self.samples), len(corners)))

        def _maybe_auto_collect(self, corners, gray):
            if os.environ.get("CALIB_GUI_DEBUG"):
                self._ac_calls = getattr(self, "_ac_calls", 0) + 1
            if not self.auto_collect.get() or corners is None or gray is None:
                return
            now = time.time()
            if now - self._last_save < 0.4:
                if os.environ.get("CALIB_GUI_DEBUG"):
                    self._ac_interval = getattr(self, "_ac_interval", 0) + 1
                return
            if len(self.samples) >= self.max_samples:
                if not getattr(self, "_cap_warned", False):
                    self._cap_warned = True
                    self.log("样本已达上限 %d 张（约 %d MB）：覆盖度不会再涨。"
                             "先点【标定】，或点【应用尺寸】清空重来。"
                             % (self.max_samples, self.sample_bytes // (1024 * 1024)))
                return
            if self._last_corners is not None and len(self._last_corners) == len(corners):
                shift = float(np.linalg.norm(
                    corners.reshape(-1, 2) - self._last_corners.reshape(-1, 2), axis=1).mean())
                if shift < self.min_shift:
                    if os.environ.get("CALIB_GUI_DEBUG"):
                        self._ac_shift = getattr(self, "_ac_shift", 0) + 1
                        self._ac_last_shift = shift
                    return
            s_new = Sample(gray, corners)
            self.samples.append(s_new)
            self.sample_bytes += s_new.arr.nbytes
            self._record_cover(corners, gray.shape)
            self._last_corners = corners
            self._last_save = now
            self.log("自动采集第 %d 张" % len(self.samples))

        # ---------------- 标定 / 检验 / 输出 ----------------
        def _record_cover(self, corners, shape):
            sz = (int(round(self.vars["sw"].get())), int(round(self.vars["sh"].get())))
            prm = coverage_params(corners, (shape[1], shape[0]), sz)
            if prm:
                self.sample_params.append(prm)

        def do_calibrate(self):
            if getattr(self, "_calib_busy", False):
                self.log("上一次标定还在算，请稍候 ...")
                return
            if len(self.samples) < MIN_SAMPLES:
                self.log("样本不足（%d 张，至少要 %d 张）：点【开始采集】多摆几个姿态"
                         % (len(self.samples), MIN_SAMPLES))
                return
            square = self.vars["sq"].get() / 1000.0
            sz = (int(round(self.vars["sw"].get())), int(round(self.vars["sh"].get())))
            samples = list(self.samples)
            self._calib_busy = True
            self._calib_result = None
            self._last_hint = None
            self.log("标定中 ...（%d 张样本，棋盘格 %dx%d，方格 %.1fmm）"
                     "要逐张重新检测角点，可能要几秒到几十秒，窗口不会卡死"
                     % (len(samples), sz[0], sz[1], square * 1000))
            self.hint.set("标定计算中，请稍候 ...（窗口可以继续动）")
            self.root.update_idletasks()

            def work():
                try:
                    self._calib_result = ("ok", calibrate_samples(samples, sz, square, k=2))
                except Exception as e:  # noqa: BLE001
                    self._calib_result = ("err", str(e))

            threading.Thread(target=work, daemon=True).start()

        def _demo_switch_to(self, target):
            self.drv_var.set(DRIVERS[target]["label"])
            self.on_driver_change()
            self.log("自测：开始启动 %s ..." % DRIVERS[target]["label"])
            self.start_driver()

        def _poll_calibration(self):
            """后台标定算完后回到主线程更新界面。"""
            if not getattr(self, "_calib_busy", False) or self._calib_result is None:
                return
            kind, payload = self._calib_result
            self._calib_result = None
            self._calib_busy = False
            if kind == "err":
                self.log("标定失败: %s（建议再采几张不同姿态，并确认尺寸/方格填对）" % payload)
                return
            self.res = payload
            self.checked = False
            self.saved = False
            r = self.res
            self.log("标定完成：RMS %.4f px，fx=%.3f fy=%.3f cx=%.3f cy=%.3f"
                     % (r["rms"], r["K"][0, 0], r["K"][1, 1], r["K"][0, 2], r["K"][1, 2]))
            if r.get("n_rejected"):
                self.log("已自动剔除 %d 张坏帧（运动模糊/误检），剩余 %d 张有效"
                         % (r["n_rejected"], r["n_views"]))
            jd = "优秀" if r["rms"] < 0.3 else ("合格" if r["rms"] < 0.5 else "不合格")
            self.res_var.set("RMS %.4f px (%s)\nfx=%.2f  fy=%.2f\n覆盖 %.1f%%  视图 %d"
                             % (r["rms"], jd, r["K"][0, 0], r["K"][1, 1],
                                r["coverage"] * 100, r["n_views"]))
            self.build_maps()
            if getattr(self, "_demo_pending", False):
                self._demo_pending = False
                self.do_check()
                self.do_save()

        def build_maps(self):
            import cv2
            if not self.res:
                self.mapx = self.mapy = None
                return
            R = np.eye(3)
            ncm, _ = cv2.getOptimalNewCameraMatrix(self.res["K"], self.res["D"],
                                                   self.res["image_size"], 0.0)
            self.mapx, self.mapy = cv2.initUndistortRectifyMap(
                self.res["K"], self.res["D"], R, ncm, self.res["image_size"], cv2.CV_32FC1)
            self._maps = (self.mapx, self.mapy, self.res["image_size"])
            with self.lock:
                self.state["maps"] = self._maps

        def do_check(self):
            if not self.res:
                self.log("还没标定，先点“标定”")
                return
            square = self.vars["sq"].get() / 1000.0
            sz = (int(round(self.vars["sw"].get())), int(round(self.vars["sh"].get())))
            text = build_report(self.driver, self.res, square, sz, self.topic)
            self.session = self.session or new_session("%s_%dx%d" % (self.driver, sz[0], sz[1]))
            with open(os.path.join(self.session, "检验报告.txt"), "w") as f:
                f.write(text + "\n")
            self.checked = True
            self.log("检验完成，报告已存 %s/检验报告.txt" % os.path.basename(self.session))
            self._show_report(text)

        def _show_report(self, text):
            win = tk.Toplevel(self.root)
            win.title("检验报告")
            txt = tk.Text(win, width=58, height=28,
                          font=(getattr(self, "mono_family", None) or "TkFixedFont", 12))
            txt.grid(row=0, column=0, sticky="nsew")
            sb = tk.Scrollbar(win, command=txt.yview)
            sb.grid(row=0, column=1, sticky="ns")
            txt.configure(yscrollcommand=sb.set)
            txt.insert("1.0", text)
            txt.configure(state="disabled")
            tk.Button(win, text="关闭", command=win.destroy).grid(row=1, column=0, pady=6)

        def do_save(self):
            if not self.res:
                self.log("还没标定，先点“标定”")
                return
            square = self.vars["sq"].get() / 1000.0
            sz = (int(round(self.vars["sw"].get())), int(round(self.vars["sh"].get())))
            self.session = save_outputs(self.driver, self.res, self.samples, sz, square,
                                        session=self.session,
                                        ace_template=self.ace_template)
            # 本次运行的驱动日志一并归档（原始日志在 <结果根>/驱动日志/ 下）
            logf = getattr(self, "driver_log", None)
            if logf and os.path.isfile(logf):
                try:
                    import shutil as _sh
                    _sh.copyfile(logf, os.path.join(self.session, "驱动日志.log"))
                except OSError:
                    pass
            self.saved = True
            self.log("已输出到 %s" % self.session)
            self.log("  %s" % ", ".join(sorted(os.listdir(self.session))))

        def _build_ace_json(self):
            """按 ACE 模板渲染当前标定结果，返回 JSONC 文本。"""
            res = self.res
            sz = (int(round(self.vars["sw"].get())), int(round(self.vars["sh"].get())))
            square = self.vars["sq"].get() / 1000.0
            header = ace_json.build_header(
                when=time.strftime("%Y-%m-%d %H:%M:%S"),
                driver=DRIVERS[self.driver]["label"], pattern=sz,
                square_mm=square * 1000.0, size=res["image_size"],
                n_views=res["n_views"], rms=res["rms"],
                uv=res.get("uv_range"), session=ace_json.rel_to_home(self.session or ""))
            model = "plumb_bob" if res["D"].size <= 5 else "rational_polynomial"
            return ace_json.build(self.ace_template, res["K"], res["D"], res["P"],
                                  res["image_size"], model, header)

        def do_export_ace(self):
            """把当前内参单独导出成 ACE27 相机配置 JSON（另存为）。"""
            if not self.res:
                self.log("还没标定，先点“标定”")
                return
            if not os.path.isfile(self.ace_template):
                self.log("ACE 模板不存在：%s（可用 --ace-template 指定）" % self.ace_template)
                return
            from tkinter import filedialog
            default_name = ace_json.default_output_name(self.ace_template)
            path = filedialog.asksaveasfilename(
                title="导出 ACE 相机配置 JSON", initialdir=self.session or HERE,
                initialfile=default_name, defaultextension=".json",
                filetypes=[("JSON 配置", "*.json"), ("全部文件", "*.*")])
            if not path:
                return
            try:
                text = self._build_ace_json()
            except Exception as e:                                        # noqa: BLE001
                self.log("生成 ACE 配置失败：%s" % e)
                return
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
            self.log("已导出 ACE 配置：%s（模板 %s，仅内参字段不同）"
                     % (path, os.path.basename(self.ace_template)))

        def do_write_driver(self):
            if not self.res:
                self.log("还没标定，先点“标定”")
                return
            d = DRIVERS[self.driver]
            cfg = os.path.join(d["ws"], "config", "camera_info.yaml")
            if not os.path.isfile(cfg):
                self.log("找不到驱动内参文件 %s" % cfg)
                return
            if not self.session or not os.path.isfile(os.path.join(self.session, "camera_info.yaml")):
                self.do_save()
            with open(cfg) as f:
                old = f.read()
            with open(cfg + ".bak." + time.strftime("%Y%m%d_%H%M%S"), "w") as f:
                f.write(old)
            with open(os.path.join(self.session, "camera_info.yaml")) as f:
                new = f.read()
            new = re.sub(r"^camera_name:.*", "camera_name: " + d["cam_name"], new, flags=re.M)
            with open(cfg, "w") as f:
                f.write(new)
            home = os.path.expanduser("~/.ros/camera_info")
            os.makedirs(home, exist_ok=True)
            with open(os.path.join(home, d["pkg"] + ".yaml"), "w") as f:
                f.write(new)
            self.log("已写入驱动 %s（原文件已备份 .bak.*）" % cfg)

        def apply_size(self):
            sz = (int(round(self.vars["sw"].get())), int(round(self.vars["sh"].get())))
            self.samples.clear()
            self.sample_params = []
            self.sample_bytes = 0
            self._cap_warned = False
            self.res = None
            self.mapx = self.mapy = None
            self._maps = None
            with self.lock:
                self.state["maps"] = None
                self.state["disp_rgb"] = None
            self._last_corners = None
            self._last_param_sent.clear()
            self.checked = False
            self.saved = False
            self.res_var.set("尚未标定")
            with self.lock:
                self.state["size"] = sz
            self.log("尺寸已设为 %dx%d 内角点，方格 %.0fmm；已清空样本，重新开始采集"
                     % (sz[0], sz[1], self.vars["sq"].get()))

        # ---------------- 引导 / 按钮可用性 ----------------
        def _driver_running(self):
            return bool(self.driver_proc and self.driver_proc.poll() is None)

        def _current_step(self):
            """0=启动驱动 1=采集 2=标定 3=检验 4=输出 5=全部完成"""
            with self.lock:
                has_frame = self.state["bgr"] is not None
            if not (self._driver_running() or has_frame):
                return 0
            if len(self.samples) < MIN_SAMPLES:
                return 1
            if not self.res:
                return 2
            if not self.checked:
                return 3
            if not self.saved:
                return 4
            return 5

        def _check_publishers(self):
            """话题上有多个发布者时（比如另开了别的相机节点），图像会交替来自不同相机，
            标定结果直接作废。这里定时检查并给出显眼警告。"""
            now = time.time()
            if now - getattr(self, "_pub_t", 0) < 5.0:
                return
            self._pub_t = now
            try:
                infos = self.cam.node.get_publishers_info_by_topic(self.topic)
            except Exception:
                return
            if len(infos) > 1:
                names = sorted({i.node_name for i in infos})
                self._pub_warn = ("话题 %s 上有 %d 个发布者（%s）：图像会交替来自不同相机，"
                                  "标定结果不可信！请只保留一个相机节点。"
                                  % (self.topic, len(infos), "、".join(names)))
                if self._pub_warn != getattr(self, "_pub_warn_logged", None):
                    self._pub_warn_logged = self._pub_warn
                    self.log("⚠ " + self._pub_warn)
            else:
                if getattr(self, "_pub_warn", None):
                    self.log("话题 %s 的发布者恢复为 1 个，可以正常标定了" % self.topic)
                self._pub_warn = None
                self._pub_warn_logged = None

        def _update_coverage(self):
            prog = coverage_progress(self.sample_params)
            for key, lab, pv, _lo, _hi in prog:
                cv, pc = self.cov_bars[key]
                cv.delete("all")
                wpx = max(1, cv.winfo_width())
                fill = int(round(pv * wpx))
                if fill > 0:
                    cv.create_rectangle(0, 0, fill, 24, fill=_cover_color(pv), outline="")
                pc.configure(text="%3d%%" % round(pv * 100))
            return prog

        def _refresh(self):
            """定时刷新：步骤条配色 + 按钮灰化 + 下一步提示（提示变了才写日志）"""
            step = self._current_step()
            for i, (name, _t) in enumerate(STEPS):
                bg = "#d4edda" if i < step else ("#cce5ff" if i == step else "#ededed")
                self.step_labels[name].configure(background=bg)
            with self.lock:
                has_frame = self.state["bgr"] is not None
                ncorner = self.state["ncorner"]
            running = self._driver_running()
            busy = getattr(self, "_calib_busy", False)
            decode_err = self.state.get("decode_err")
            det_ok = ncorner > 0 and (time.time() - self._last_corner_ts) < 2.5

            def set_state(key, ok):
                self.btns[key].configure(state="normal" if ok else "disabled")

            self.btn_start.configure(state="disabled" if running else "normal")
            self.btn_stop.configure(state="normal" if running else "disabled")
            set_state("collect", has_frame and not busy)
            set_state("grab", has_frame and ncorner > 0 and not busy)
            set_state("calib", len(self.samples) >= MIN_SAMPLES and not busy)
            set_state("check", self.res is not None)
            set_state("save", self.res is not None)
            set_state("ace", self.res is not None)
            set_state("write", self.res is not None)

            n = len(self.samples)
            prog = coverage_progress(self.sample_params)
            worst = min(prog, key=lambda t: t[2]) if prog else None
            cover_note = ""
            if self.sample_params and worst and worst[2] < 1.0:
                cover_note = "（覆盖度：%s 只到 %d%%，补些%s不同的姿态）" % (
                    worst[1], round(worst[2] * 100),
                    "位置" if worst[0] in ("x", "y") else ("距离远近" if worst[0] == "size" else "倾斜角度"))
            if getattr(self, "_pub_warn", None):
                hint = "⚠ " + self._pub_warn
            elif decode_err:
                hint = "图像的像素格式不支持（%s）→ 把驱动改成 rgb8 或 mono8 后重试" % decode_err
            elif busy:
                hint = "标定计算中，请稍候 ...（窗口可以继续动，算完会自动出结果）"
            elif step == 0:
                hint = ("第 1 步：点【启动驱动】启动相机（相机插好、USB3 口直连）。"
                        "相机已在别处启动的话，等画面出现即可。")
            elif not det_ok:
                hint = ("画面里没检测到标定板 → 确认板子完整在画面内、光线均匀；"
                        "若尺寸不对（12x9 个方格应为 11x8），改滑条/输入框后点【应用尺寸并重新开始】。")
            elif n < MIN_SAMPLES:
                hint = ("第 2 步：把标定板完整放进画面，点【开始采集】开始采集；"
                        "已 %d/%d 张（ROS 标准建议 %d 张），覆盖画面四角四边并加倾斜。"
                        % (n, MIN_SAMPLES, RECOMMEND_SAMPLES))
            elif step == 1:
                good = n >= GOODENOUGH_SAMPLES or all(p[2] >= 1.0 for p in prog)
                if not good:
                    hint = ("已 %d 张，还没到 ROS 的“够”（%d 张或覆盖度四条全绿），"
                            "再采几张更稳，然后点【标定】。%s"
                            % (n, GOODENOUGH_SAMPLES, cover_note))
                else:
                    hint = ("已 %d 张、覆盖度达标，可以点【标定】了。%s"
                            % (n, cover_note))
            elif step == 2:
                hint = "第 3 步：点【标定】计算内参（RMS < 0.5px 合格，< 0.3px 优秀）"
            elif step == 3:
                hint = "第 4 步：点【检验（图形报告）】查看精度报告；不满意就继续采集后重新点【标定】"
            elif step == 4:
                hint = ("第 5 步：点【Save to results dir】归档；"
                        "要让驱动正式使用这份内参，再点【Write to driver】")
            else:
                hint = "完成 ✔ 重新标定：改尺寸滑条/输入框 →【应用尺寸并重新开始】→ 回到第 2 步"
            try:
                self.hint_label.configure(
                    foreground="#c00000" if getattr(self, "_pub_warn", None) else "#0645ad")
            except Exception:
                pass
            if hint != self._last_hint:
                self._last_hint = hint
                self.hint.set(hint)
                self.log(("⚠ " if getattr(self, "_pub_warn", None) else "【下一步】") + hint)

        # ---------------- 主循环 ----------------
        def tick(self):
            if getattr(self, "_want_quit", False):
                self.quit()
                return
            try:
                with self.lock:
                    self.state["undist"] = bool(self.show_undist.get())
                    self.state["maps"] = getattr(self, "_maps", None)
                    native = self.state.get("native")
                if native and native != self._native_size:
                    self._native_size = native
                    try:
                        c = self.img_canvas
                        self._on_img_resize(type("E", (), {"width": c.winfo_width(),
                                                           "height": c.winfo_height()})())
                    except Exception:
                        pass
                if native:
                    per = max(1, native[0] * native[1])      # 一张灰度图字节数
                    cap = int(SAMPLE_MEMORY_BUDGET / per)
                    self.max_samples = max(MIN_SAMPLES, min(MAX_SAMPLES, cap))
                if time.time() - getattr(self, "_t0", 0) > 4.0:
                    self._diag_once()
                if os.environ.get("CALIB_GUI_DEBUG"):
                    self._ac_stats(time.time())
                self._drain_log()
                self._update_image()
                self._poll_calibration()
                self._check_publishers()
                self._update_coverage()
                self._refresh()
            except Exception as e:
                self.log("界面异常: %s" % e)
            self.root.after(80, self.tick)

        def _ac_stats(self, now):
            t = getattr(self, "_ac_t", 0.0)
            if not t:
                self._ac_t = now
                return
            if now - t < 5.0:
                return
            self._ac_t = now
            self.log("[调试] 5秒: 收到帧 %d, 处理后 %d, 检出成功 %d, UI tick %.1fHz, 自动采集调用 %d, "
                     "间隔不足 %d, 位移不足 %d (最近位移 %.1f), 成功 %d 张"
                     % (getattr(self.cam, "_img_in", 0), getattr(self.cam, "_img_ok", 0),
                        getattr(self.cam, "_img_det", 0),
                        len(self._rate) / 2.0, getattr(self, "_ac_calls", 0),
                        getattr(self, "_ac_interval", 0), getattr(self, "_ac_shift", 0),
                        getattr(self, "_ac_last_shift", -1), len(self.samples)))
            self._ac_calls = self._ac_interval = self._ac_shift = 0
            self.cam._img_in = self.cam._img_ok = self.cam._img_det = 0

        def _diag_once(self):
            if not os.environ.get("CALIB_GUI_DEBUG") or getattr(self, "_diag_done", False):
                return
            self._diag_done = True
            try:
                import tkinter as _tk
                worst = []
                def walk(w, depth=0):
                    try:
                        worst.append((w.winfo_reqwidth(), w.winfo_reqheight(),
                                      w.winfo_class(), str(w)[:60]))
                    except Exception:
                        pass
                    for c in w.winfo_children():
                        walk(c, depth + 1)
                walk(self.root)
                worst.sort(reverse=True)
                self.log("[诊断] 窗口 req=%dx%d 实际=%dx%d"
                         % (self.root.winfo_reqwidth(), self.root.winfo_reqheight(),
                            self.root.winfo_width(), self.root.winfo_height()))
                for rw, rh, cls, name in worst[:6]:
                    self.log("[诊断] 最宽控件 %5d x %-5d %-10s %s" % (rw, rh, cls, name))
                worst.sort(key=lambda t: -t[1])
                for rw, rh, cls, name in worst[:4]:
                    self.log("[诊断] 最高控件 %5d x %-5d %-10s %s" % (rw, rh, cls, name))
            except Exception as e:
                self.log("[诊断] 失败 %s" % e)

        def _drain_log(self):
            n = 0
            while n < 30:
                try:
                    line = self.logq.get_nowait()
                except queue.Empty:
                    break
                self.log_text.insert("end", line + "\n")
                n += 1
            if n:
                self.log_text.see("end")

        def _on_img_resize(self, event):
            """可用区域变化时，取其中最大的“相机宽高比”矩形，既放得最大又没有黑边。"""
            AW, AH = max(160, min(int(event.width), 2600)), max(120, min(int(event.height), 1800))
            nw, nh = self._native_size or (4, 3)
            ar = (nw / float(nh)) if nh else (4 / 3.0)
            if AW / float(AH) > ar:          # 区域偏宽 -> 以高度为准
                ih = AH
                iw = int(round(AH * ar))
            else:                            # 区域偏高 -> 以宽度为准
                iw = AW
                ih = int(round(AW / ar))
            # 限制总像素数（窗口很大时按比例缩一点，保证贴图流畅）
            if iw * ih > MAX_RENDER_PIXELS:
                f2 = (MAX_RENDER_PIXELS / float(iw * ih)) ** 0.5
                iw, ih = int(iw * f2), int(ih * f2)
            if os.environ.get("CALIB_GUI_DEBUG") and (iw, ih) != getattr(self, "_want_size", None):
                self.log("[调试] 图像显示区 -> %dx%d（可用 %dx%d）" % (iw, ih, AW, AH))
            self._want_size = (iw, ih)
            with self.lock:
                self.state["want_size"] = (iw, ih)

        def _update_image(self):
            with self.lock:
                rgb = self.state["disp_rgb"]
                corners = self.state["corners"]
                ncorner = self.state["ncorner"]
                stamp = self.state["stamp"]
            if rgb is None:
                err = self.state.get("decode_err")
                if err:
                    self.status.set("收到图像但解码失败：%s" % err)
                else:
                    self.status.set("等待 %s 的图像 ...（先点“启动驱动”，或确认相机已插好）" % self.topic)
                return
            now = time.time()
            self._rate = [t for t in self._rate if now - t < 2.0] + [now]
            fps = (len(self._rate) - 1) / 2.0
            if stamp == self._last_stamp:
                # 没有新帧就只更新状态栏，不重画（省 CPU）
                self._set_status(ncorner, corners, fps, rgb.shape[1], rgb.shape[0])
                return
            self._last_stamp = stamp

            h, w = rgb.shape[:2]
            cw = max(1, self.img_canvas.winfo_width())
            ch = max(1, self.img_canvas.winfo_height())
            # 复用同一个 PhotoImage（尺寸不变时用 paste，比每帧新建快很多）
            if self._photo is None or self._photo.width() != w or self._photo.height() != h:
                self._photo = ImageTk.PhotoImage(Image.fromarray(rgb))
                self.img_canvas.delete("img")
                self._img_item = self.img_canvas.create_image(
                    cw // 2, ch // 2, image=self._photo, anchor="center", tags="img")
            else:
                self._photo.paste(Image.fromarray(rgb))
                self.img_canvas.coords(self._img_item, cw // 2, ch // 2)

            if corners is not None:
                self._last_corner_ts = now
            self._set_status(ncorner, corners, fps, w, h)
            if corners is not None:
                self._maybe_auto_collect(corners, self.state["gray"])

        def _set_status(self, ncorner, corners, fps, w, h):
            native = self._native_size or (w, h)
            self.status.set("相机 %dx%d | 显示 %dx%d | %.1f Hz | 棋盘格 %dx%d | 已采集 %d/%d 张 | 检测: %s"
                            % (native[0], native[1], w, h, fps,
                               round(self.vars["sw"].get()), round(self.vars["sh"].get()),
                               len(self.samples), self.max_samples,
                               ("✔ %d 角点" % ncorner) if corners is not None else "✘ 未检测到"))

        def demo_finish(self):
            self.log("演示：开始标定")
            if self.auto_collect.get():
                self.toggle_collect()
            if not self.res:
                self._demo_pending = True      # 标定是后台线程，算完再自动检验+输出
            self.do_calibrate()

        def quit(self):
            if getattr(self, "_quitting", False):
                return
            self._quitting = True
            self.stop_driver()
            try:
                self.cam.stop()
                self.cam.join(timeout=1.5)      # 等采集线程干净退出，避免退出时崩溃
            except Exception:
                pass
            try:
                import rclpy
                rclpy.shutdown()
            except Exception:
                pass
            try:
                self.root.destroy()
            except Exception:
                pass

    import rclpy
    rclpy.init()
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")   # 默认主题下滑条轨道几乎看不见
    except Exception:
        pass
    mono_family = setup_fonts(root)
    setup_style(root, ttk, UI_FONT, 12)
    app = App(root)
    app.mono_family = mono_family
    # 控件都建好后再定尺寸，避免被最小请求尺寸顶掉
    _sw, _sh = root.winfo_screenwidth(), root.winfo_screenheight()
    _want_geom = app._geom or "%dx%d+%d+%d" % (min(_sw - 40, 2400), min(_sh - 70, 1620), 10, 10)
    root.geometry(_want_geom)
    root.after(400, lambda: root.geometry(_want_geom))   # 控件请求稳定后再定一次
    if os.environ.get("CALIB_GUI_DEBUG"):
        print("[调试] 窗口请求尺寸 %dx%d，实际 %dx%d"
              % (root.winfo_reqwidth(), root.winfo_reqheight(),
                 root.winfo_width(), root.winfo_height()), flush=True)

    # 被外部 SIGTERM/SIGINT 干掉时也走一遍清理，避免驱动子进程变野节点
    import signal as _signal

    def _on_signal(_signum, _frame):
        # 信号处理函数里不能操作 Tk（会段错误）：只置标志，交给主循环处理
        app._want_quit = True

    for _sig in (_signal.SIGTERM, _signal.SIGINT):
        try:
            _signal.signal(_sig, _on_signal)
        except Exception:
            pass

    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.quit()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


# =========================================================================== #
# 无相机自检：用合成图画走完 检测→标定→检验→输出
# =========================================================================== #
def self_test(size, square):
    import tempfile
    sys.path.insert(0, HERE)
    from selftest_synthetic import COLS, ROWS, SQ, K_GT, D_GT, make_dataset

    size, square = (COLS, ROWS), SQ          # 合成数据集固定为 8x6 内角点 / 25mm
    tmp = tempfile.mkdtemp(prefix="gui_selftest_")
    imgdir = os.path.join(tmp, "images")
    print("1) 合成标定图（真值 fx=%.2f, k1=%.5f）" % (K_GT[0, 0], D_GT[0]))
    make_dataset(imgdir)

    samples = []
    for name in sorted(os.listdir(imgdir)):
        gray = cv2.imread(os.path.join(imgdir, name), cv2.IMREAD_GRAYSCALE)
        corners, _obj, n = detect_in(gray, size)
        if corners is not None:
            samples.append(Sample(gray, corners))
    print("2) 检测：%d 张图，采到 %d 个样本（每张 %d 角点）" % (len(os.listdir(imgdir)), len(samples),
                                                              len(samples[0].corners) if samples else 0))
    assert samples, "一张都没检测到"

    print("3) 标定 ...")
    res = calibrate_samples(samples, size, square, k=2)
    print("    RMS=%.4f px  fx=%.3f  fy=%.3f  cx=%.3f  cy=%.3f  覆盖=%.1f%%"
          % (res["rms"], res["K"][0, 0], res["K"][1, 1], res["K"][0, 2], res["K"][1, 2],
             res["coverage"] * 100))

    report = build_report("hik", res, square, size, TOPIC)
    print("4) 检验报告：")
    for line in report.splitlines()[9:16]:
        print("    " + line)

    session = save_outputs("hik", res, samples, size, square,
                           session=os.path.join(tmp, "session"),
                           ace_template=ace_json.DEFAULT_TEMPLATE)
    files = sorted(os.listdir(session))
    print("5) 输出目录 %s\n    %s" % (session, ", ".join(files)))

    # ACE27 相机配置：与模板比对，除内参字段外必须逐项一致
    ace_name = ace_json.default_output_name(ace_json.DEFAULT_TEMPLATE)
    ace_ok, ace_detail = False, "模板不存在，跳过"
    if ace_name in files:
        try:
            with open(ace_json.DEFAULT_TEMPLATE, encoding="utf-8") as f:
                tpl = json.loads(ace_json.strip_comments(f.read()))
            with open(os.path.join(session, ace_name), encoding="utf-8") as f:
                got = json.loads(ace_json.strip_comments(f.read()))
            cam_key = list(tpl["cameras"].keys())[0]
            diffs = ace_json._leaf_diffs(tpl, got)
            allowed = {"image_width", "image_height", "distortion_model",
                       "camera_matrix", "distortion_coefficients", "projection_matrix"}
            bad = [d for d in diffs if not (len(d) > 3 and d[0] == "cameras" and d[2] in allowed)]
            ace_ok = not bad
            ace_detail = ("%d 处内参差异" % len(diffs)) if ace_ok else ("多出差异 %s" % bad)
        except Exception as e:                                        # noqa: BLE001
            ace_detail = "解析失败: %s" % e
    print("    ACE 配置：%s —— %s" % (ace_name, ace_detail))

    checks = [
        ("检测到样本 >= 12", len(samples) >= 12),
        ("RMS < 0.35 px", res["rms"] < 0.35),
        ("fx 误差 < 3 px", abs(res["K"][0, 0] - K_GT[0, 0]) < 3.0),
        ("k1 相对误差 < 5%%", abs(res["D"].ravel()[0] - D_GT[0]) / abs(D_GT[0]) < 0.05),
        ("覆盖 > 40%%", res["coverage"] > 0.4),
        ("报告含 RMS 结论", "RMS 重投影误差" in report),
        ("输出 camera_info.yaml", "camera_info.yaml" in files),
        ("输出 检验报告.txt", "检验报告.txt" in files),
        ("输出 frames/", "frames" in files),
        ("输出 去畸变对比.jpg", "去畸变对比.jpg" in files),
        ("输出 ACE 配置 %s" % ace_name, ace_name in files),
        ("ACE 配置仅内参不同", ace_ok),
    ]
    ok = True
    print("6) 断言")
    for name, passed in checks:
        print("   [%s] %s" % ("通过" if passed else "失败", name))
        ok &= bool(passed)
    print("\n==> 界面核心逻辑自检 %s" % ("全部通过" if ok else "存在失败项"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
