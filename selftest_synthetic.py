#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
合成数据自检：用已知内参渲染带畸变的棋盘格图片，再跑 calibrate_mono.py，
验证恢复出来的内参与畸变系数是否正确、YAML 格式是否符合 ROS 规范。

  python3 selftest_synthetic.py
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

W, H = 1440, 1080
K_GT = np.array([[1750.34699, 0.0, 733.30737],
                 [0.0, 1749.40469, 549.10270],
                 [0.0, 0.0, 1.0]])
D_GT = np.array([-0.056721, 0.044929, -0.000139, -0.000374, 0.0])
COLS, ROWS = 8, 6           # 内角点数
SQ = 0.025                  # 方格边长 (m)
BOARD_W, BOARD_H = (COLS + 1) * SQ, (ROWS + 1) * SQ

# 每个"畸变图像素"对应的真实光线方向（固定内参，只算一次）。
# 关键：畸变图上像素 p 的光线是 undistort(p)，不能用 initUndistortRectifyMap 正着套。
_uu, _vv = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
_grid = np.stack([_uu.ravel(), _vv.ravel()], axis=1).reshape(-1, 1, 2)
_norm = cv2.undistortPoints(_grid, K_GT, D_GT).reshape(H, W, 2)
_XN = _norm[..., 0]
_YN = _norm[..., 1]
_ONES = np.ones_like(_XN)


def render(rvec, tvec, seed=0, noise=1.5):
    """渲染一张带畸变的棋盘格图（黑白棋盘 + 白底），返回 uint8 灰度图。"""
    R, _ = cv2.Rodrigues(rvec)
    n = R[:, 2]                                   # 标定板平面法向（相机系）
    d = float(n @ tvec)                           # 平面方程 n·X = d
    ray = np.stack([_XN, _YN, _ONES], axis=-1)    # (H,W,3)
    denom = ray @ n
    s = d / np.where(np.abs(denom) < 1e-9, 1e-9, denom)
    pcam = ray * s[..., None]
    pboard = (pcam - tvec) @ R                    # R^T (pcam - t)
    u, v = pboard[..., 0], pboard[..., 1]

    iu = np.floor(u / SQ)
    iv = np.floor(v / SQ)
    inside = (u >= 0) & (u < BOARD_W) & (v >= 0) & (v < BOARD_H)
    black = ((iu + iv) % 2 == 0) & inside
    field = np.where(black, 0.0, 255.0).astype(np.float32)

    # 每个像素的光线已在 _XN/_YN 中去畸变，直接得到的就是带畸变的图像
    img = field
    if noise:
        img = img + np.random.default_rng(1000 + seed).normal(0, noise, img.shape)
    img = np.clip(img, 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(img, (3, 3), 0.6)


def visible(rvec, tvec, margin=40):
    """检查标定板外框是否完整落在画面内（留出边距）。"""
    pts = np.array([[0, 0, 0], [BOARD_W, 0, 0], [0, BOARD_H, 0], [BOARD_W, BOARD_H, 0]], np.float32)
    proj, _ = cv2.projectPoints(pts, rvec, tvec, K_GT, D_GT)
    proj = proj.reshape(-1, 2)
    return (proj[:, 0] > margin).all() and (proj[:, 0] < W - margin).all() and \
           (proj[:, 1] > margin).all() and (proj[:, 1] < H - margin).all()


def make_dataset(outdir, n_views=20, seed=7):
    rng = np.random.default_rng(seed)
    os.makedirs(outdir, exist_ok=True)
    center = np.array([BOARD_W / 2, BOARD_H / 2, 0.0])
    made = 0
    tries = 0
    while made < n_views and tries < 4000:
        tries += 1
        rvec = np.array([rng.uniform(-0.36, 0.36), rng.uniform(-0.36, 0.36), rng.uniform(-0.55, 0.55)])
        u0 = rng.uniform(0.2 * W, 0.8 * W)
        v0 = rng.uniform(0.2 * H, 0.8 * H)
        depth = rng.uniform(0.40, 1.10)
        xn, yn = (u0 - K_GT[0, 2]) / K_GT[0, 0], (v0 - K_GT[1, 2]) / K_GT[1, 1]
        cam_center = depth * np.array([xn, yn, 1.0])
        R, _ = cv2.Rodrigues(rvec)
        tvec = cam_center - R @ center
        if not visible(rvec, tvec):
            continue
        img = render(rvec, tvec, seed=made)
        cv2.imwrite(os.path.join(outdir, "view_%02d.png" % made), img)
        made += 1
    if made < n_views:
        raise RuntimeError("只生成了 %d 张图" % made)
    return made


def parse_ros_yaml(path):
    text = open(path).read()
    def grab(key):
        m = re.search(key + r":\s*\n\s*rows:\s*(\d+)\s*\n\s*cols:\s*(\d+)\s*\n\s*data:\s*\[([^\]]*)\]", text)
        rows, cols, data = int(m.group(1)), int(m.group(2)), [float(x) for x in m.group(3).split(",")]
        return np.array(data).reshape(rows, cols)
    return {
        "width": int(re.search(r"image_width:\s*(\d+)", text).group(1)),
        "height": int(re.search(r"image_height:\s*(\d+)", text).group(1)),
        "name": re.search(r"camera_name:\s*(\S+)", text).group(1),
        "model": re.search(r"distortion_model:\s*(\S+)", text).group(1),
        "K": grab("camera_matrix"),
        "D": grab("distortion_coefficients").ravel(),
        "R": grab("rectification_matrix"),
        "P": grab("projection_matrix"),
        "text": text,
    }


def homography_residual(corners, cols, rows):
    """
    平面标定板的角点在针孔+去畸变后必须严格满足单应关系（透视只把平面直线映成直线）。
    畸变不是单应变换，所以没去畸变时残差会明显更大 —— 这是判断标定能不能用的硬指标。
    """
    from calibrate_mono import matching_points
    obj = matching_points((cols, rows), SQ, "chessboard")[:, :2].astype(np.float64)
    dst = corners.reshape(-1, 2).astype(np.float64)
    H, _ = cv2.findHomography(obj, dst, 0)
    proj = cv2.perspectiveTransform(obj.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(proj - dst, axis=1)
    return float(err.mean()), float(err.max())


def geometric_check(imgdir, yaml_path, cols=COLS, rows=ROWS):
    """用标定结果去畸变后，角点是否共线 —— 检验标定能不能真正用。"""
    sys.path.insert(0, HERE)
    from undistort_with_yaml import load_camera_info, make_maps
    from calibrate_mono import detect_corners

    info = load_camera_info(yaml_path)
    mapx, mapy = make_maps(info)
    img = cv2.imread(os.path.join(imgdir, "view_00.png"), cv2.IMREAD_GRAYSCALE)
    und = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
    cv2.imwrite(os.path.join(os.path.dirname(yaml_path), "undistorted_view00.png"), und)

    raw_c, _ = detect_corners(img, (cols, rows), "chessboard")
    und_c, _ = detect_corners(und, (cols, rows), "chessboard")
    if raw_c is None or und_c is None:
        return None
    raw_mean, raw_max = homography_residual(raw_c, cols, rows)
    und_mean, und_max = homography_residual(und_c, cols, rows)
    print("5) 单应一致性（检验 YAML 里的 K/D/R/P 与去畸变链路自洽；单位 px）")
    print("   参考：未去畸变 %.3f（单块小板近似满足单应，故该值本身不大）" % raw_mean)
    print("   去畸变后: 平均 %.3f  最大 %.3f" % (und_mean, und_max))
    return raw_mean, und_mean


def main():
    tmp = tempfile.mkdtemp(prefix="calib_selftest_")
    try:
        imgdir = os.path.join(tmp, "images")
        print("1) 用已知内参合成 %d 张带畸变棋盘格图 -> %s" % (20, imgdir))
        n = make_dataset(imgdir)
        print("   已生成 %d 张" % n)

        out = os.path.join(tmp, "hik.yaml")
        cmd = [sys.executable, os.path.join(HERE, "calibrate_mono.py"),
               "--images", imgdir, "--size", "%dx%d" % (COLS, ROWS), "--square", str(SQ),
               "--camera-name", "hik_test", "--output", out, "--min-views", "12", "--k", "2"]
        print("2) 运行:", " ".join(cmd[1:]))
        proc = subprocess.run(cmd, capture_output=True, text=True)
        print(proc.stdout)
        if proc.returncode != 0:
            print(proc.stderr)
            return 1

        y = parse_ros_yaml(out)
        K, D = y["K"], y["D"]
        fx_err = abs(K[0, 0] - K_GT[0, 0])
        fy_err = abs(K[1, 1] - K_GT[1, 1])
        cx_err = abs(K[0, 2] - K_GT[0, 2])
        cy_err = abs(K[1, 2] - K_GT[1, 2])
        rms = float(re.search(r"RMS 重投影误差:\s*([0-9.]+)", proc.stdout).group(1))

        print("3) 校验结果")
        print("   fx %.3f (真值 %.3f, 误差 %.3f px)" % (K[0, 0], K_GT[0, 0], fx_err))
        print("   fy %.3f (真值 %.3f, 误差 %.3f px)" % (K[1, 1], K_GT[1, 1], fy_err))
        print("   cx %.3f (真值 %.3f, 误差 %.3f px)" % (K[0, 2], K_GT[0, 2], cx_err))
        print("   cy %.3f (真值 %.3f, 误差 %.3f px)" % (K[1, 2], K_GT[1, 2], cy_err))
        print("   k1 %.6f (真值 %.6f) | k2 %.6f (真值 %.6f)"
              % (D[0], D_GT[0], D[1], D_GT[1]))
        print("   RMS %.4f px | 模型 %s | 尺寸 %dx%d | 名字 %s"
              % (rms, y["model"], y["width"], y["height"], y["name"]))
        print("   YAML 头部:")
        for line in y["text"].splitlines()[:4]:
            print("     " + line)

        checks = [
            ("fx 误差 < 3 px", fx_err < 3.0),
            ("fy 误差 < 3 px", fy_err < 3.0),
            ("cx 误差 < 5 px", cx_err < 5.0),
            ("cy 误差 < 5 px", cy_err < 5.0),
            ("k1 相对误差 < 5%%", abs(D[0] - D_GT[0]) / abs(D_GT[0]) < 0.05),
            ("RMS < 0.35 px", rms < 0.35),
            ("distortion_model == plumb_bob", y["model"] == "plumb_bob"),
            ("distortion_coefficients 为 1x5", D.size == 5),
            ("rectification_matrix == I", np.allclose(y["R"], np.eye(3))),
            ("projection_matrix 为 3x4", y["P"].shape == (3, 4)),
            ("projection_matrix 左上 3x3 == getOptimalNewCameraMatrix(K,D,size,0)",
             np.allclose(y["P"][:3, :3],
                         cv2.getOptimalNewCameraMatrix(K, D, (W, H), 0.0)[0], atol=1e-4)),
            ("image_width/height 正确", (y["width"], y["height"]) == (W, H)),
            ("camera_name 正确", y["name"] == "hik_test"),
        ]
        print("4) 断言")
        ok = True
        for name, passed in checks:
            print("   [%s] %s" % ("通过" if passed else "失败", name))
            ok &= bool(passed)

        geo = geometric_check(imgdir, out)
        if geo is None:
            print("   [失败] 去畸变后仍检测不到棋盘格")
            ok = False
        else:
            raw_mean, und_mean = geo
            for name, passed in [("去畸变后单应残差 < 0.4 px（YAML 与去畸变链路自洽）", und_mean < 0.4)]:
                print("   [%s] %s" % ("通过" if passed else "失败", name))
                ok &= bool(passed)

        print("\n==> 自检 %s" % ("全部通过" if ok else "存在失败项"))
        return 0 if ok else 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
