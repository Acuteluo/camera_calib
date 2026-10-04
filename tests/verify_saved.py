"""从磁盘重新读取 GUI 产物，独立核对内参精度、格式和图片。"""
import json
import re
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ace_json

session = Path(sys.argv[1]).resolve(strict=True)
with (session / "camera_info.yaml").open() as f:
    info = yaml.safe_load(f)
K = np.array(info["camera_matrix"]["data"]).reshape(3, 3)
D = np.array(info["distortion_coefficients"]["data"])
P = np.array(info["projection_matrix"]["data"]).reshape(3, 4)
assert (info["image_width"], info["image_height"]) == (1440, 1080)
assert abs(K[0, 0] - 1750.34699) < 10, K
assert abs(K[1, 1] - 1749.40469) < 10, K
assert abs(D[0] + 0.056721) < 0.01, D
assert np.allclose(P[:, :3], cv2.getOptimalNewCameraMatrix(K, D, (1440, 1080), 0)[0], atol=0.01)
frames = sorted((session / "frames").glob("*.png"))
assert len(frames) >= 12, len(frames)
assert all(cv2.imread(str(p), 0).shape == (1080, 1440) for p in frames)
assert cv2.imread(str(session / "去畸变对比.jpg")).shape == (1080, 2880, 3)
report = (session / "检验报告.txt").read_text()
assert float(re.search(r"RMS 重投影误差\s+([0-9.]+)", report).group(1)) < 0.35, report
assert (session / "说明.txt").stat().st_size > 0
fixture = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).parent / "fixtures" / "ace_camera.json"
template = json.loads(ace_json.strip_comments(fixture.read_text()))
actual = json.loads(ace_json.strip_comments((session / fixture.name).read_text()))
allowed = {"image_width", "image_height", "distortion_model", "camera_matrix",
           "distortion_coefficients", "projection_matrix"}
assert all(len(p) > 3 and p[0] == "cameras" and p[2] in allowed
           for p in ace_json._leaf_diffs(template, actual))
camera = actual["cameras"]["camera_0"]
assert np.allclose(camera["camera_matrix"]["data"], K.ravel())
assert np.allclose(camera["distortion_coefficients"]["data"], D, atol=1e-6)
assert np.allclose(camera["projection_matrix"]["data"], P.ravel(), atol=1e-5)
print("保存验证通过：%d 张图、YAML、报告、去畸变对比、ACE JSON；fx=%.3f fy=%.3f" %
      (len(frames), K[0, 0], K[1, 1]))
