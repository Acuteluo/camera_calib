"""环境回归：PATH/PYTHONHOME 污染和实际导入错误。"""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

HERE = Path(__file__).resolve().parents[1]


class RuntimeTests(unittest.TestCase):
    def test_system_python_with_polluted_environment(self):
        with tempfile.TemporaryDirectory() as tmp:
            fake = Path(tmp) / "python3"
            fake.write_text("#!/bin/sh\necho wrong-interpreter >&2\nexit 99\n")
            fake.chmod(0o755)
            env = {**os.environ, "PATH": tmp + ":" + os.environ["PATH"],
                   "PYTHONHOME": "/nonexistent-python-home", "PYTHONPATH": tmp}
            proc = subprocess.run(
                ["bash", "-c", 'source "$1/runtime.sh"; calib_base_env; '
                 '"$CALIB_PYTHON" -s -c "import sys; print(sys.executable)"', "test", str(HERE)],
                env=env, capture_output=True, text=True)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertEqual(proc.stdout.strip(), "/usr/bin/python3")
            self.assertNotIn("wrong-interpreter", proc.stderr)

    def test_import_error_is_not_hidden(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "cv2.py").write_text('raise ImportError("libopencv-test.so missing")\n')
            proc = subprocess.run([sys.executable, "-s", str(HERE / "check_runtime.py")],
                                  env={**os.environ, "PYTHONPATH": tmp},
                                  capture_output=True, text=True)
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn("libopencv-test.so missing", proc.stderr)
            self.assertIn("cv2 (python3-opencv)", proc.stderr)

    def test_topic_switch_is_deferred_to_camera_thread(self):
        from calib_gui import CameraThread
        cam = CameraThread("/old", {}, None)
        cam.node = mock.Mock()
        cam.set_topic("/new")
        self.assertEqual(cam.topic, "/new")
        cam.node.destroy_subscription.assert_not_called()

    def test_camera_thread_can_join(self):
        from calib_gui import CameraThread
        class OfflineCamera(CameraThread):
            def run(self):
                pass
        cam = OfflineCamera("/unused", {}, None)
        cam.start()
        cam.stop()
        cam.join(timeout=1)
        self.assertFalse(cam.is_alive())


if __name__ == "__main__":
    unittest.main()
