#!/usr/bin/env python3
"""验证实际解释器能加载依赖，保留原始错误（不仅检查包名）。"""
import argparse
import importlib
import sys

BASE = {"numpy": "python3-numpy", "cv2": "python3-opencv",
        "yaml": "python3-yaml", "PIL.Image": "python3-pil",
        "PIL.ImageTk": "python3-pil.imagetk", "tkinter": "python3-tk"}
ROS = {"rclpy": "ROS rclpy", "sensor_msgs.msg": "ROS sensor_msgs"}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ros", action="store_true")
    args = ap.parse_args()
    print("Python: %s (%s)" % (sys.executable, sys.version.split()[0]))
    failed = False
    for name, package in {**BASE, **(ROS if args.ros else {})}.items():
        try:
            mod = importlib.import_module(name)
            if name == "cv2" and not hasattr(mod, "findChessboardCornersSB"):
                raise RuntimeError("需要 OpenCV 4.5+ 的 findChessboardCornersSB")
            print("  ✔ %s %s — %s" % (name, getattr(mod, "__version__", ""),
                                      getattr(mod, "__file__", "")))
        except Exception as exc:
            failed = True
            print("  ✘ %s (%s): %s: %s" % (name, package, type(exc).__name__, exc),
                  file=sys.stderr)
    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
