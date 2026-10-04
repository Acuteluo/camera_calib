import tempfile
from pathlib import Path
from unittest import mock
import unittest

import cv2
import numpy as np
import calib_gui as gui


class CalibrationLogicTests(unittest.TestCase):
    def test_sessions_do_not_overwrite_within_same_second(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(gui, "RESULTS_ROOT", tmp), \
                mock.patch.object(gui.time, "strftime", return_value="20261004_190000_"):
            first = Path(gui.new_session("hik"))
            (first / "sentinel").write_text("keep")
            second = Path(gui.new_session("hik"))
            self.assertNotEqual(first, second)
            self.assertEqual((first / "sentinel").read_text(), "keep")
            self.assertEqual((Path(tmp) / "latest").resolve(), second)

    def test_rejects_mixed_resolution_and_empty_samples(self):
        a = gui.Sample(np.zeros((100, 100), np.uint8), None)
        b = gui.Sample(np.zeros((80, 100), np.uint8), None)
        for samples in ([], [a, b]):
            with self.assertRaises(ValueError):
                gui.calibrate_samples(samples, (8, 6), 0.025)

    def test_linear_error_is_measured_per_view_in_pixels(self):
        from calibrate_mono import matching_points
        board = matching_points((8, 6), 0.025, "chessboard")
        K = np.array([[700., 0, 320], [0, 700, 240], [0, 0, 1]])
        D = np.zeros((1, 5))
        samples = []
        points = []
        for i in range(4):
            c, _ = cv2.projectPoints(board, np.array([0.1*i, 0.03, 0.2*i]),
                                     np.array([-0.1, -0.08 + 0.02*i, 0.7]), K, D)
            points.append(c.astype(np.float32))
            samples.append(gui.Sample(np.zeros((480, 640), np.uint8), c.copy()))
        with mock.patch.object(gui, "calibrate", return_value=(0., K, D, [0.]*4)), \
                mock.patch.object(cv2, "cornerSubPix", side_effect=lambda gray, c, *args: c):
            result = gui.calibrate_samples(samples, (8, 6), 0.025)
        self.assertLess(result["linear"], 1e-4)
        for sample, original in zip(samples, points):
            np.testing.assert_array_equal(sample.corners, original)

    def test_asymmetric_grid_scales_only_once(self):
        from calibrate_mono import matching_points
        points = matching_points((3, 2), 0.025, "acircles")
        np.testing.assert_allclose(points[:, :2], [[0, 0], [.05, 0], [.1, 0],
                                                   [.025, .025], [.075, .025], [.125, .025]], atol=1e-8)

    @unittest.skipUnless(hasattr(cv2, "aruco"), "当前 OpenCV 不包含 aruco")
    def test_charuco_board_with_old_and_new_opencv(self):
        from calibrate_mono import make_charuco_board, detect_corners
        board, dictionary = make_charuco_board((8, 6), 0.025, 0.012, "5x5_250")
        image = (board.generateImage((900, 700)) if hasattr(board, "generateImage")
                 else board.draw((900, 700)))
        corners, objects = detect_corners(image, (8, 6), "charuco", charuco=(board, dictionary))
        self.assertIsNotNone(corners)
        self.assertGreaterEqual(len(corners), 8)
        self.assertEqual(len(corners), len(objects))

    def test_save_uses_topic_and_reports_image_write_failure(self):
        result = dict(K=np.eye(3), D=np.zeros(5), P=np.hstack([np.eye(3), np.zeros((3, 1))]),
                      image_size=(100, 100), rms=0.1, coverage=0.8, linear=0.1,
                      n_views=4, per_view=[0.1]*4)
        sample = gui.Sample(np.zeros((100, 100), np.uint8), None)
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(cv2, "imwrite", return_value=False):
            with self.assertRaises(OSError):
                gui.save_outputs("hik", result, [sample], (8, 6), 0.025,
                                 session=tmp, topic="/test/image_raw")
            self.assertIn("/test/image_raw", (Path(tmp) / "检验报告.txt").read_text())


if __name__ == "__main__":
    unittest.main()
