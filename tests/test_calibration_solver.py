"""用同一批多姿态含噪角点，对比加速求解与原 SVD 的标定精度。"""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import calibrate_mono


def make_views(count=20):
    width, height = 1440, 1080
    cols, rows, square = 11, 8, 0.025
    obj = np.zeros((rows * cols, 3), np.float32)
    gx, gy = np.meshgrid(np.arange(cols), np.arange(rows))
    obj[:, 0] = gx.ravel() * square
    obj[:, 1] = gy.ravel() * square

    k_true = np.array([[1750.34699, 0.0, 733.30737],
                       [0.0, 1749.40469, 549.10270],
                       [0.0, 0.0, 1.0]], np.float64)
    d_true = np.array([-0.056721, 0.044929, -0.000139, -0.000374, 0.0], np.float64)
    rng = np.random.default_rng(20261014)
    object_points, image_points = [], []

    attempts = 0
    while len(object_points) < count and attempts < count * 100:
        attempts += 1
        rvec = np.array([rng.uniform(-0.52, 0.52), rng.uniform(-0.52, 0.52),
                         rng.uniform(-0.70, 0.70)], np.float64)
        depth = rng.uniform(0.78, 1.55)
        u0 = rng.uniform(0.17 * width, 0.83 * width)
        v0 = rng.uniform(0.17 * height, 0.83 * height)
        center = np.array([(u0 - k_true[0, 2]) * depth / k_true[0, 0],
                           (v0 - k_true[1, 2]) * depth / k_true[1, 1], depth], np.float64)
        rotation, _ = cv2.Rodrigues(rvec)
        board_center = np.array([(cols - 1) * square / 2,
                                 (rows - 1) * square / 2, 0.0], np.float64)
        tvec = center - rotation @ board_center
        projected, _ = cv2.projectPoints(obj, rvec, tvec, k_true, d_true)
        projected = projected.reshape(-1, 2)
        if (projected[:, 0].min() < 35 or projected[:, 0].max() > width - 35 or
                projected[:, 1].min() < 35 or projected[:, 1].max() > height - 35):
            continue
        noisy = projected + rng.normal(0.0, 0.20, projected.shape)
        object_points.append(obj.copy())
        image_points.append(noisy.astype(np.float32).reshape(-1, 1, 2))

    if len(object_points) != count:
        raise RuntimeError("Could not generate enough in-frame calibration views")
    return (width, height), object_points, image_points


def svd_baseline(object_points, image_points, image_size, flags):
    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None, flags=flags)
    per_view = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
        per_view.append(float(np.linalg.norm(
            projected.reshape(-1, 2) - img.reshape(-1, 2), axis=1).mean()))
    return rms, camera_matrix, distortion, per_view


class CalibrationSolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original_threads = cv2.getNumThreads()
        cv2.setNumThreads(1)
        cls.image_size, cls.object_points, cls.image_points = make_views()
        print("[solver regression] OpenCV %s; threads=%d" %
              (cv2.__version__, cv2.getNumThreads()))

    @classmethod
    def tearDownClass(cls):
        cv2.setNumThreads(cls.original_threads)

    def test_qr_matches_default_svd_without_changing_distortion_flags(self):
        args = SimpleNamespace(k=2, zero_tangent_dist=False,
                               fix_principal_point=False, fix_aspect_ratio=False)
        distortion_flags = calibrate_mono.calibration_flags(args)
        baseline = svd_baseline(self.object_points, self.image_points,
                                self.image_size, distortion_flags)
        actual = calibrate_mono.calibrate(self.object_points, self.image_points,
                                         self.image_size, distortion_flags)

        self.assertAlmostEqual(actual[0], baseline[0], delta=1e-5)
        np.testing.assert_allclose(actual[1], baseline[1], rtol=0, atol=1e-4)
        np.testing.assert_allclose(actual[2], baseline[2], rtol=0, atol=1e-6)
        np.testing.assert_allclose(actual[3], baseline[3], rtol=0, atol=1e-5)
        self.assertEqual(actual[2].ravel()[4], 0., "k=2 时 k3 应继续固定为零")

    def test_invalid_empty_input_still_raises(self):
        with self.assertRaises(cv2.error):
            calibrate_mono.calibrate([], [], self.image_size, 0)


if __name__ == "__main__":
    unittest.main()
