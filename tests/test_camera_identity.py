"""设备身份只能来自采集时的真实设备；模板值不能冒充新相机序列号。"""
import tempfile
from pathlib import Path
import unittest

import numpy as np

from calib_gui import Sample, build_report, read_camera_identity, save_outputs


class CameraIdentityTests(unittest.TestCase):
    @staticmethod
    def add_usb(root, port, vendor, product_id, model, serial):
        path = root / port
        path.mkdir()
        for name, value in {"idVendor": vendor, "idProduct": product_id,
                            "manufacturer": "Hikrobot", "product": model,
                            "serial": serial, "speed": "5000"}.items():
            (path / name).write_text(value + "\n")

    def test_unique_camera_is_reported_and_snapshot_survives_unplug(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.add_usb(root, "4-2", "2bdf", "0001", "MV-CS016-10UC", "SN-NEW-42")
            self.add_usb(root, "3-9", "3277", "00a0", "Webcam", "WEB-1")
            identity = read_camera_identity("hik", tmp)
            self.assertEqual(identity["status"], "confirmed")
            self.assertEqual(identity["devices"][0]["serial"], "SN-NEW-42")
            for path in (root / "4-2").iterdir():
                path.unlink()
            (root / "4-2").rmdir()
            result = dict(K=np.eye(3), D=np.zeros(5), image_size=(1440, 1080),
                          rms=.2, coverage=.7, linear=.1, n_views=12,
                          per_view=[.2] * 12, camera_identity=identity)
            report = build_report("hik", result, .025, (11, 8), "/image_raw")
            self.assertIn("ACE device_id: SN-NEW-42", report)
            self.assertIn("ACE vid_pid  : 2bdf:0001", report)
            self.assertNotIn("WEB-1", report)
            with tempfile.TemporaryDirectory() as out:
                result["image_size"] = (32, 32)
                result["P"] = np.hstack([np.eye(3), np.zeros((3, 1))])
                save_outputs("hik", result, [Sample(np.zeros((32, 32), np.uint8), None)],
                             (11, 8), .025, session=out)
                self.assertIn("SN-NEW-42", (Path(out) / "检验报告.txt").read_text())
                self.assertIn("SN-NEW-42", (Path(out) / "说明.txt").read_text())

    def test_multiple_same_vendor_cameras_are_not_guessed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.add_usb(root, "4-2", "2bdf", "0001", "Camera A", "A-1")
            self.add_usb(root, "4-3", "2bdf", "0001", "Camera B", "B-2")
            identity = read_camera_identity("hik", tmp)
            self.assertEqual(identity["status"], "multiple")
            self.assertEqual([d["serial"] for d in identity["devices"]], ["A-1", "B-2"])
            self.assertEqual(read_camera_identity("galaxy", tmp)["status"], "unavailable")


if __name__ == "__main__":
    unittest.main()
