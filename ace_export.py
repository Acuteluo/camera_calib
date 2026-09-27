#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键把已有标定结果批量转换成 ACE27 相机配置 JSON（JSONC）。

读取每个标定结果目录里的 `camera_info.yaml`（内参 / 畸变 / 投影矩阵）与 `说明.txt`
（时间 / 驱动 / 棋盘格 / 分辨率 / 有效视图 / RMS），以指定的 ACE 相机配置为模板渲染。
除内参相关字段外，模板内容**逐字节保持原样**（安装外参、采集参数、注释、缩进都不动）。

安全约定
--------
* 只**新增**文件，从不删除、从不移动既有文件；
* 目标已存在同名 JSON 时跳过，只有显式 `--force` 才覆盖；
* `--dry-run` 只打印计划，不写盘。

输出位置
--------
* 默认：写回各自的结果目录，文件名与模板同名（如 `uav.json`），可直接拷回
  `ACE27/config/cameras/`；
* `--out DIR`：改为统一写到 DIR，文件名用结果目录名（避免互相覆盖）。

用法
----
    python3 ace_export.py --dry-run                 # 先看计划
    python3 ace_export.py                           # 写回各自结果目录（跳过已存在）
    python3 ace_export.py --out /tmp/ace_json       # 统一输出到别处
    python3 ace_export.py --template ~/ACE27/config/cameras/hero.json
    python3 ace_export.py --uv                       # 顺带从 frames/ 统计角点覆盖范围
"""

import argparse
import os
import re
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ace_json  # noqa: E402

DEFAULT_RESULTS = os.path.join(HERE, "标定结果")


# --------------------------------------------------------------------------- #
# 结果目录解析
# --------------------------------------------------------------------------- #
def parse_info_txt(path):
    """解析 `说明.txt`，返回 {键: 值}；文件不存在返回空字典。"""
    info = {}
    if not os.path.isfile(path):
        return info
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if ":" in line:
                k, v = line.split(":", 1)
                info[k.strip()] = v.strip()
    return info


def _first_pair(text):
    """从字符串里取第一组 `A x B` 数字，返回 (A, B) 或 None。"""
    m = re.search(r"(\d+)\s*[xX×]\s*(\d+)", text or "")
    return (int(m.group(1)), int(m.group(2))) if m else None


def load_camera_info(path):
    """读 camera_info.yaml，返回 (K, D, P, (w, h), model)。"""
    with open(path, encoding="utf-8") as f:
        ci = yaml.safe_load(f)
    K = np.array(ci["camera_matrix"]["data"], float).reshape(3, 3)
    D = np.array(ci["distortion_coefficients"]["data"], float).ravel()
    P = np.array(ci["projection_matrix"]["data"], float).reshape(3, 4)
    size = (int(ci["image_width"]), int(ci["image_height"]))
    return K, D, P, size, ci.get("distortion_model", "plumb_bob")


def corner_uv(frames_dir, pattern):
    """从 frames/*.png 统计角点覆盖范围 u/v（可选，慢）。返回 (u0,u1,v0,v1) 或 None。

    在全分辨率帧上检测：与标定时用的角点口径一致，统计出的 u/v 才和工具报告同源。
    """
    import cv2
    import glob
    files = sorted(glob.glob(os.path.join(frames_dir, "left-*.png")))
    if not files:
        return None
    us, vs = [], []
    for f in files:
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        ok, corners = cv2.findChessboardCornersSB(g, pattern, flags=cv2.CALIB_CB_ACCURACY)
        if not ok:
            continue
        c = corners.reshape(-1, 2)
        us.append(c[:, 0])
        vs.append(c[:, 1])
    if not us:
        return None
    u = np.concatenate(us)
    v = np.concatenate(vs)
    return (int(round(u.min())), int(round(u.max())),
            int(round(v.min())), int(round(v.max())))


def describe_session(session_dir, template_text, template_name, with_uv=False):
    """把结果目录渲染成 JSONC 文本；不可转换时返回 (None, 原因)。"""
    ci_path = os.path.join(session_dir, "camera_info.yaml")
    if not os.path.isfile(ci_path):
        return None, "没有 camera_info.yaml（未标定）"
    K, D, P, size, model = load_camera_info(ci_path)

    info = parse_info_txt(os.path.join(session_dir, "说明.txt"))
    pattern = _first_pair(info.get("棋盘格", "")) or (11, 8)
    square_mm = 0.0
    m = re.search(r"方格\s*([0-9.]+)", info.get("棋盘格", ""))
    if m:
        square_mm = float(m.group(1))
    n_views = int(re.search(r"\d+", info.get("有效视图", "0")).group()) if info.get("有效视图") else 0
    rms = float(re.search(r"[0-9.]+", info.get("RMS", "0")).group()) if info.get("RMS") else 0.0

    uv = None
    if with_uv:
        uv = corner_uv(os.path.join(session_dir, "frames"), pattern)

    header = ace_json.build_header(
        when=info.get("时间", "-"), driver=info.get("驱动", "-"),
        pattern=pattern, square_mm=square_mm, size=size, n_views=n_views, rms=rms,
        uv=uv, session=ace_json.rel_to_home(session_dir))
    text = ace_json.render(template_text, K, D, P, size, model, header)
    return text, None


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def iter_sessions(results_dir):
    """列出结果目录下的标定会话（跳过软链接与非目录）。"""
    if not os.path.isdir(results_dir):
        return []
    out = []
    for name in sorted(os.listdir(results_dir)):
        p = os.path.join(results_dir, name)
        if os.path.islink(p) or not os.path.isdir(p):
            continue
        out.append(p)
    return out


def main():
    ap = argparse.ArgumentParser(
        description="把已有标定结果批量转换成 ACE27 相机配置 JSON（只新增，不删除）")
    ap.add_argument("--results", default=DEFAULT_RESULTS, help="标定结果根目录")
    ap.add_argument("--template", default=ace_json.DEFAULT_TEMPLATE,
                    help="ACE 相机配置模板（JSONC），默认 ACE27/config/cameras/uav.json")
    ap.add_argument("--out", default=None,
                    help="统一输出目录；不给则写回各自结果目录（文件名同模板）")
    ap.add_argument("--name", default=None, help="输出文件名；默认取模板文件名")
    ap.add_argument("--force", action="store_true", help="目标已存在时覆盖")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不写盘")
    ap.add_argument("--uv", action="store_true", help="从 frames/ 统计角点覆盖范围写进注释（慢）")
    args = ap.parse_args()

    if not os.path.isfile(args.template):
        print("模板不存在: %s" % args.template)
        return 2
    with open(args.template, encoding="utf-8") as f:
        template_text = f.read()
    out_name = args.name or ace_json.default_output_name(args.template)

    sessions = iter_sessions(args.results)
    if not sessions:
        print("在 %s 下没找到标定结果目录" % args.results)
        return 1

    print("模板    : %s" % args.template)
    print("输出名  : %s" % out_name)
    print("结果目录: %s（%d 个会话）" % (args.results, len(sessions)))
    if args.out:
        print("输出位置: %s（统一输出）" % args.out)
    else:
        print("输出位置: 各结果目录内（只新增，已存在则跳过）")
    print("-" * 66)

    done, skipped, failed = 0, 0, 0
    for s in sessions:
        name = os.path.basename(s)
        target = (os.path.join(args.out, name + ".json") if args.out
                  else os.path.join(s, out_name))
        if os.path.exists(target) and not args.force:
            print("  跳过  %-34s 已有 %s" % (name, os.path.basename(target)))
            skipped += 1
            continue
        try:
            text, why = describe_session(s, template_text, out_name, args.uv)
        except Exception as e:                                        # noqa: BLE001
            print("  失败  %-34s %s" % (name, e))
            failed += 1
            continue
        if text is None:
            print("  跳过  %-34s %s" % (name, why))
            skipped += 1
            continue
        if args.dry_run:
            print("  待写  %-34s -> %s（%d 字节）" % (name, target, len(text)))
            done += 1
            continue
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, "w", encoding="utf-8") as f:
            f.write(text)
        print("  已生成 %-33s -> %s" % (name, target))
        done += 1

    print("-" * 66)
    print("完成：生成 %d，跳过 %d，失败 %d%s"
          % (done, skipped, failed, "（dry-run，未写盘）" if args.dry_run else ""))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
