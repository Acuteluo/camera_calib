"""验证两个入口使用一致 SDK，并拒绝过期的兼容驱动。"""
import hashlib
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import driver_runtime as runtime
from calib_gui import camera_identity_lines, camera_identity_summary


class DriverRuntimeTests(unittest.TestCase):
    def test_overlay_sdk_wins_over_old_workspace_and_is_deduplicated(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(runtime, 'HERE', Path(tmp)):
            base = Path(tmp)
            install = base / '.driver_runtime' / 'hik' / 'install'
            install.mkdir(parents=True)
            (install / 'setup.bash').touch()
            patches = base / 'driver_patches'
            patches.mkdir()
            (patches / 'hik.patch').write_text('validated patch')
            (install.parent / 'patch.sha256').write_text(hashlib.sha256(b'validated patch').hexdigest())
            lib = str(install / 'hik_camera' / 'lib')
            env = {'LD_LIBRARY_PATH': '/old/workspace:' + lib + ':/ros'}
            result = runtime.sdk_environment('hik', env)
            self.assertEqual(result['LD_LIBRARY_PATH'], lib + ':/old/workspace:/ros')
            self.assertEqual(env['LD_LIBRARY_PATH'], '/old/workspace:' + lib + ':/ros')
            (patches / 'hik.patch').write_text('new patch')
            with self.assertRaisesRegex(RuntimeError, '补丁已更新'):
                runtime.sdk_environment('hik', env)

    def test_explicit_missing_sdk_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(runtime, 'HERE', Path(tmp)):
            with self.assertRaisesRegex(RuntimeError, 'HIK_SDK_ROOT'):
                runtime.sdk_environment('hik', {'HIK_SDK_ROOT': tmp})

    def test_gige_report_uses_selected_serial_without_usb_identity(self):
        lines = '\n'.join(camera_identity_lines(dict(status='confirmed', devices=[
            dict(serial='GIGE-42', model='GigE model', transport='GigE')])) )
        self.assertIn('GIGE-42', lines)
        self.assertIn('GigE', lines)
        self.assertNotIn('VID:PID', lines)
        summary = camera_identity_summary(dict(serial='GIGE-42', model='GigE model', transport='GigE'))
        self.assertIn('GIGE-42', summary)
        self.assertIn('GigE', summary)


if __name__ == '__main__':
    unittest.main()
