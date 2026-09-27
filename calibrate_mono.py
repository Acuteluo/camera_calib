#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
棋盘格（或圆点阵 / ChArUco）单目相机标定，直接产出 ROS camera_info 格式 YAML。

输出字段与 ROS 2 `camera_calibration` 包 cameracalibrator 完全一致：
  image_width / image_height / camera_name / camera_matrix /
  distortion_model / distortion_coefficients /
  rectification_matrix / projection_matrix

典型用法：
  python3 calibrate_mono.py --images ./images --size 8x6 --square 0.025 \
      --camera-name hik_mv --output hik_mv.yaml --save-undistorted

注意 --size 是“内角点数”，即宽 x 高方向的内部黑白交叉点个数。
一块 9x7 个方格的棋盘 -> --size 8x6；方格边长用米，例如 25 mm -> --square 0.025
"""

import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

SUB_PIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-4)


# --------------------------------------------------------------------------- #
# 标定板几何
# --------------------------------------------------------------------------- #
def parse_size(text):
    try:
        cols, rows = text.lower().split("x")
        return int(cols), int(rows)
    except Exception:
        raise argparse.ArgumentTypeError("--size 需要形如 8x6（内角点数，宽x高）")


def matching_points(pattern_size, square, pattern):
    """返回整块标定板的物点坐标（z=0 平面），与检测到的角点顺序一致。"""
    cols, rows = pattern_size
    objp = np.zeros((rows * cols, 3), np.float32)
    if pattern == "acircles":
        # 非对称圆点阵：每个方格中心一个点，奇偶行错开半个方格
        for i in range(rows):
            for j in range(cols):
                objp[i * cols + j, 0] = (2 * j + (i % 2)) * square
                objp[i * cols + j, 1] = i * square
    else:
        xv, yv = np.meshgrid(np.arange(cols), np.arange(rows))
        objp[:, 0] = xv.ravel()
        objp[:, 1] = yv.ravel()
    return objp * float(square)


# --------------------------------------------------------------------------- #
# 角点检测
# --------------------------------------------------------------------------- #
def make_charuco_board(pattern_size, square, marker_size, dict_name):
    dict_id = getattr(cv2.aruco, "DICT_" + dict_name.upper(), cv2.aruco.DICT_5X5_250)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    board = cv2.aruco.CharucoBoard(
        (pattern_size[0] + 1, pattern_size[1] + 1), float(square), float(marker_size), dictionary
    )
    return board, dictionary


def detect_corners(gray, pattern_size, pattern, use_sb=True, charuco=None):
    """返回 (image_points Nx1x2 float32, object_points Nx3 float32) 或 (None, None)。"""
    if pattern == "charuco":
        board, dictionary = charuco
        if hasattr(cv2.aruco, "CharucoDetector"):
            detector = cv2.aruco.CharucoDetector(board)
            ch_corners, ch_ids, _, _ = detector.detectBoard(gray)
        else:  # OpenCV < 4.7 旧 API
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
            if ids is None or len(ids) < 4:
                return None, None
            _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
        if ch_ids is None or len(ch_ids) < 8:
            return None, None
        all_obj = board.getChessboardCorners()
        ids = ch_ids.ravel().astype(int)
        return (
            ch_corners.reshape(-1, 1, 2).astype(np.float32),
            all_obj[ids].reshape(-1, 3).astype(np.float32),
        )

    if pattern == "chessboard":
        if use_sb and hasattr(cv2, "findChessboardCornersSB"):
            flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
            ok, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=flags)
            if ok:
                return corners.reshape(-1, 1, 2).astype(np.float32), matching_points(
                    pattern_size, 1.0, pattern
                )
            return None, None
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if not ok:
            return None, None
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUB_PIX_CRITERIA)
        return corners.astype(np.float32), matching_points(pattern_size, 1.0, pattern)

    flag = (
        cv2.CALIB_CB_SYMMETRIC_GRID
        if pattern == "circles"
        else cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING
    )
    ok, corners = cv2.findCirclesGrid(gray, pattern_size, flags=flag)
    if not ok:
        return None, None
    return corners.astype(np.float32), matching_points(pattern_size, 1.0, pattern)


# --------------------------------------------------------------------------- #
# 标定
# --------------------------------------------------------------------------- #
def calibration_flags(args):
    flags = 0
    k = args.k
    if k > 3:
        flags |= cv2.CALIB_RATIONAL_MODEL
    for idx, name in ((6, "CALIB_FIX_K6"), (5, "CALIB_FIX_K5"), (4, "CALIB_FIX_K4"),
                      (3, "CALIB_FIX_K3"), (2, "CALIB_FIX_K2"), (1, "CALIB_FIX_K1")):
        if k < idx:
            flags |= getattr(cv2, name)
    if args.zero_tangent_dist:
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
    if args.fix_principal_point:
        flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if args.fix_aspect_ratio:
        flags |= cv2.CALIB_FIX_ASPECT_RATIO
    return flags


def calibrate(objpoints, imgpoints, image_size, flags):
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None, flags=flags
    )
    per_view = []
    for i, (op, ip) in enumerate(zip(objpoints, imgpoints)):
        proj, _ = cv2.projectPoints(op, rvecs[i], tvecs[i], K, D)
        err = np.linalg.norm(proj.reshape(-1, 2) - ip.reshape(-1, 2), axis=1)
        per_view.append(float(err.mean()))
    return rms, K, D, per_view


# --------------------------------------------------------------------------- #
# ROS camera_info YAML 输出
# --------------------------------------------------------------------------- #
def dist_model_name(D):
    return "rational_polynomial" if D.size > 5 else "plumb_bob"


def _fmt_mat(mat, precision):
    text = np.array2string(np.asarray(mat, dtype=float), precision=precision,
                           suppress_small=True, separator=", ")
    text = text.replace("[", "").replace("]", "").replace("\n", "\n        ")
    return "[%s]" % text


def ros_yaml(camera_name, K, D, R, P, image_size):
    return "\n".join([
        "image_width: %d" % image_size[0],
        "image_height: %d" % image_size[1],
        "camera_name: " + camera_name,
        "camera_matrix:",
        "  rows: 3",
        "  cols: 3",
        "  data: " + _fmt_mat(K, 5),
        "distortion_model: " + dist_model_name(D),
        "distortion_coefficients:",
        "  rows: 1",
        "  cols: %d" % D.size,
        "  data: [%s]" % ", ".join("%8f" % x for x in np.asarray(D).flat),
        "rectification_matrix:",
        "  rows: 3",
        "  cols: 3",
        "  data: " + _fmt_mat(R, 8),
        "projection_matrix:",
        "  rows: 3",
        "  cols: 4",
        "  data: " + _fmt_mat(P, 5),
        "",
    ])


# --------------------------------------------------------------------------- #
def collect_images(paths):
    files = []
    for p in paths:
        if os.path.isdir(p):
            for ext in ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff", "*.pgm"):
                files += glob.glob(os.path.join(p, ext))
                files += glob.glob(os.path.join(p, ext.upper()))
        else:
            files += glob.glob(p)
    return sorted(set(files))


def main():
    ap = argparse.ArgumentParser(description="棋盘格单目标定 -> ROS camera_info YAML",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", nargs="+", required=True, help="图片目录或通配符（可多个）")
    ap.add_argument("--size", type=parse_size, default=(8, 6), help="内角点数，形如 8x6（默认 8x6）")
    ap.add_argument("--square", type=float, default=0.025, help="方格边长，单位米（默认 0.025）")
    ap.add_argument("--pattern", default="chessboard",
                    choices=["chessboard", "circles", "acircles", "charuco"])
    ap.add_argument("--charuco-marker-size", type=float, default=None, help="ChArUco 标记边长（米）")
    ap.add_argument("--aruco-dict", default="5x5_250", help="ChArUco 字典，如 5x5_250")
    ap.add_argument("--camera-name", default="narrow_stereo", help="写入 YAML 的 camera_name")
    ap.add_argument("--output", "-o", default=None,
                    help="输出 YAML 路径；不给就自动存到 标定结果/<时间戳>_<名字>_<尺寸>/camera_info.yaml")
    ap.add_argument("--results-dir", default=None,
                    help="结果根目录（默认 <脚本目录>/标定结果）")
    ap.add_argument("--label", default=None, help="会话目录标签（默认用 --camera-name 和 --size）")
    ap.add_argument("--alpha", type=float, default=0.0,
                    help="projection_matrix 的裁剪系数：0=裁掉无效像素(默认)，1=保留全部视野")
    ap.add_argument("--k", type=int, default=2, help="径向畸变系数个数 1~6（默认 2，>3 用 rational 模型）")
    ap.add_argument("--min-views", type=int, default=10, help="最少有效视图数（默认 10）")
    ap.add_argument("--detect-only", action="store_true",
                    help="只做角点检测并验证 --size 是否正确，不做标定、不写 YAML")
    ap.add_argument("--no-sb", action="store_true", help="禁用 findChessboardCornersSB")
    ap.add_argument("--fix-principal-point", action="store_true")
    ap.add_argument("--fix-aspect-ratio", action="store_true")
    ap.add_argument("--zero-tangent-dist", action="store_true")
    ap.add_argument("--save-undistorted", action="store_true", help="额外保存第一张图去畸变前后对比")
    args = ap.parse_args()

    files = collect_images(args.images)
    if not files:
        sys.exit("没有找到图片，请检查 --images 路径")

    charuco = None
    if args.pattern == "charuco":
        if not args.charuco_marker_size:
            sys.exit("--pattern charuco 需要同时给出 --charuco-marker-size")
        charuco = make_charuco_board(args.size, args.square, args.charuco_marker_size, args.aruco_dict)

    objpoints, imgpoints, used, image_size = [], [], [], None
    print("== 角点检测 ==")
    for path in files:
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print("  [跳过] 读取失败 %s" % path)
            continue
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        elif (gray.shape[1], gray.shape[0]) != image_size:
            print("  [跳过] 分辨率不一致 %s" % path)
            continue
        imgp, objp = detect_corners(gray, args.size, args.pattern,
                                    use_sb=not args.no_sb, charuco=charuco)
        if imgp is None:
            print("  [失败] %s" % os.path.basename(path))
            continue
        objpoints.append(objp * (1.0 if args.pattern == "charuco" else args.square))
        imgpoints.append(imgp)
        used.append(path)
        print("  [成功] %s  角点 %d" % (os.path.basename(path), len(imgp)))

    expected = args.size[0] * args.size[1]
    if args.detect_only:
        print("\n== 尺寸验证 ==")
        print("  --size %dx%d  ->  每张图应检出 %d 个内角点" % (args.size[0], args.size[1], expected))
        n_ok = len(used)
        print("  成功 %d 张 / 共 %d 张" % (n_ok, n_ok + (len(files) - n_ok)))
        if n_ok == 0:
            print("  ✘ 一张都没检出：--size 填错了（或标定板没拍全、太模糊）。")
            print("    试试把宽高对调（%dx%d），或重新数一遍内部黑白交叉点。" % (args.size[1], args.size[0]))
            sys.exit(1)
        print("  ✔ 检出成功，说明 --size %dx%d 是对的（每张 %d 个角点）"
              % (args.size[0], args.size[1], len(imgpoints[0])))
        sys.exit(0)

    if len(used) < args.min_views:
        sys.exit("有效视图 %d 张，少于要求的 %d 张，标定不可靠。请补拍后重试。"
                 % (len(used), args.min_views))

    print("\n== 标定 ==  (%d 张视图, 图像 %dx%d)" % (len(used), image_size[0], image_size[1]))
    rms, K, D, per_view = calibrate(objpoints, imgpoints, image_size, calibration_flags(args))

    print("  整体 RMS 重投影误差: %.4f px" % rms)
    for path, err in sorted(zip(used, per_view), key=lambda x: -x[1])[:5]:
        print("    最差视图 %-32s %.4f px" % (os.path.basename(path), err))
    print("  内参 fx=%.3f fy=%.3f cx=%.3f cy=%.3f" % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
    print("  畸变 %s" % np.array2string(D.ravel(), precision=6, separator=", "))

    # 视野覆盖情况：把所有角点拼起来看铺得多开（ROS 的 X/Y/Size/Skew 进度条的等价检查）
    all_pts = np.concatenate([p.reshape(-1, 2) for p in imgpoints], axis=0)
    hull = cv2.convexHull(all_pts.astype(np.float32))
    coverage = cv2.contourArea(hull) / float(image_size[0] * image_size[1])
    print("  角点包络覆盖画面比例: %.1f%%  (建议 > 40%%，越大越好)" % (coverage * 100))

    if rms > 0.5:
        print("  [警告] RMS > 0.5 px：标定板不平、图片模糊、角点检测偏差或视角太单一。")
    if coverage < 0.3:
        print("  [警告] 覆盖比例偏低：请让标定板出现在画面四角/边缘，并加入倾斜视角。")

    R = np.eye(3)
    P = np.zeros((3, 4))
    if args.fix_principal_point or args.fix_aspect_ratio:
        ncm = K.copy()
    else:
        ncm, _ = cv2.getOptimalNewCameraMatrix(K, D, image_size, args.alpha)
    P[:3, :3] = ncm

    # ---- 结果落盘：默认 标定结果/<时间戳>_<名字>_<尺寸>/ ----
    session = None
    if args.output:
        out_path = args.output
    else:
        root = args.results_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), "标定结果")
        label = args.label or "%s_%dx%d" % (args.camera_name, args.size[0], args.size[1])
        session = os.path.join(root, time.strftime("%Y%m%d_%H%M%S_") + label)
        os.makedirs(session, exist_ok=True)
        latest = os.path.join(root, "latest")
        try:
            if os.path.islink(latest):
                os.remove(latest)
            os.symlink(os.path.abspath(session), latest)
        except OSError:
            pass
        out_path = os.path.join(session, "camera_info.yaml")
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)

    with open(out_path, "w") as f:
        f.write(ros_yaml(args.camera_name, K, D, R, P, image_size))
    print("\n已写入 ROS camera_info 格式: %s" % os.path.abspath(out_path))

    if session:
        with open(os.path.join(session, "说明.txt"), "w") as f:
            f.write("时间: %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
            f.write("命令: --images %s --size %dx%d --square %s --k %d --alpha %s\n"
                    % (" ".join(args.images), args.size[0], args.size[1], args.square, args.k, args.alpha))
            f.write("有效视图: %d / %d\n" % (len(used), len(files)))
            f.write("图像尺寸: %dx%d\n" % image_size)
            f.write("RMS 重投影误差: %.4f px\n" % rms)
            f.write("角点覆盖比例: %.1f%%\n" % (coverage * 100))
            f.write("fx=%.5f fy=%.5f cx=%.5f cy=%.5f\n" % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
            f.write("畸变 %s\n" % ", ".join("%.6f" % x for x in D.ravel()))
            f.write("\n参与标定的图片:\n")
            for i, p in enumerate(used):
                f.write("  %-40s %.4f px\n" % (os.path.basename(p), per_view[i]))
        print("  本次结果目录: %s" % session)

    if args.save_undistorted:
        gray = cv2.imread(used[0], cv2.IMREAD_COLOR)
        mapx, mapy = cv2.initUndistortRectifyMap(K, D, R, ncm, image_size, cv2.CV_32FC1)
        und = cv2.remap(gray, mapx, mapy, cv2.INTER_LINEAR)
        out = np.hstack([gray, und])
        name = os.path.splitext(out_path)[0] + "_去畸变对比.jpg"
        cv2.imwrite(name, out)
        print("已保存去畸变对比图: %s" % name)

    if session:
        print("\n结果目录内容（按时间命名，方便找）:")
        for f in sorted(os.listdir(session)):
            print("    %s" % f)


if __name__ == "__main__":
    main()
