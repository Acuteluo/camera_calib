"""新电脑上的只读帮助/缺项提示不依赖 OpenCV，也不启动相机。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'build_camera_drivers.py'


class DriverBuildTests(unittest.TestCase):
    def test_help_without_site_packages(self):
        result = subprocess.run(['/usr/bin/python3', '-S', str(SCRIPT), '--help'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--check', result.stdout)

    def test_missing_source_and_tools_are_reported_together_without_build(self):
        with tempfile.TemporaryDirectory() as tmp:
            env = dict(os.environ, CAMERA_ROOT=tmp, PATH='', MV_SDK_ROOT=tmp)
            result = subprocess.run(['/usr/bin/python3', '-S', str(SCRIPT), '--check', 'mv'],
                                    env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 1)
            self.assertIn('缺少 colcon', result.stderr)
            self.assertIn('package.xml', result.stderr)
            self.assertIn('CameraApi.h', result.stderr)
            self.assertNotIn('Traceback', result.stderr)
            self.assertEqual(list(Path(tmp).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
