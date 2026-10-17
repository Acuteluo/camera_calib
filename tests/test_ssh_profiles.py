"""Local-only tests for SSH connection profile persistence."""

import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import ssh_profiles


class SshProfileTests(unittest.TestCase):
    def test_roundtrip_applies_schema_defaults(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = ssh_profiles.save_profile(
                {"host": "camera-box", "remote_dir": "~/camera_calib"}, tmp)
            loaded = ssh_profiles.load_profile(path)

        self.assertEqual(loaded, {
            "version": 1,
            "host": "camera-box",
            "port": 22,
            "identity_file": "",
            "remote_dir": "~/camera_calib",
            "camera_root": "",
            "driver": "hik",
            "topic": "/image_raw",
        })

    def test_hosts_have_separate_files_and_same_host_updates_in_place(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "profiles"
            first = ssh_profiles.save_profile({
                "host": "user@camera-box", "port": 2222, "remote_dir": "/opt/calib",
            }, directory)
            other = ssh_profiles.save_profile({
                "host": "other-camera", "port": 2222, "remote_dir": "/opt/calib",
            }, directory)
            updated = ssh_profiles.save_profile({
                "host": "user@camera-box", "port": 2222, "remote_dir": "/srv/new-calib",
            }, directory)

            self.assertEqual(first, updated)
            self.assertNotEqual(first, other)
            self.assertEqual(len(ssh_profiles.list_profiles(directory)), 2)
            self.assertEqual(ssh_profiles.load_profile(first)["remote_dir"], "/srv/new-calib")
            self.assertEqual(ssh_profiles.load_profile(other)["host"], "other-camera")
            self.assertIn("user_camera-box", first.name)
            self.assertNotIn("@", first.name)

    def test_password_is_opt_in_and_new_directory_and_profile_are_private(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "new-profiles"
            profile = {"host": "camera-box", "remote_dir": "/opt/calib", "password": "s3cret"}
            path = ssh_profiles.save_profile(profile, directory)
            self.assertNotIn("password", ssh_profiles.load_profile(path))
            self.assertNotIn("s3cret", path.read_text(encoding="utf-8"))
            self.assertEqual(stat.S_IMODE(directory.stat().st_mode), 0o700)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)

            same_path = ssh_profiles.save_profile(profile, directory, remember_password=True)
            self.assertEqual(same_path, path)
            self.assertEqual(ssh_profiles.load_profile(path)["password"], "s3cret")
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)

    def test_existing_directory_mode_is_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "existing"
            directory.mkdir(mode=0o755)
            os.chmod(directory, 0o755)
            ssh_profiles.save_profile({"host": "camera-box", "remote_dir": "/opt/calib"}, directory)
            self.assertEqual(stat.S_IMODE(directory.stat().st_mode), 0o755)

    def test_invalid_profile_fields_and_json_are_rejected_clearly(self):
        invalid_profiles = (
            {"host": "-oProxyCommand=bad", "remote_dir": "/opt/calib"},
            {"host": "camera box", "remote_dir": "/opt/calib"},
            {"host": "camera-box", "port": True, "remote_dir": "/opt/calib"},
            {"host": "camera-box", "port": 65536, "remote_dir": "/opt/calib"},
            {"host": "camera-box", "driver": "unknown", "remote_dir": "/opt/calib"},
            {"host": "camera-box", "remote_dir": "  "},
            {"host": "camera-box", "remote_dir": "/opt/calib", "topic": "/bad topic"},
            {"host": "camera-box", "remote_dir": "/opt/calib", "identity_file": 5},
        )
        for profile in invalid_profiles:
            with self.subTest(profile=profile), self.assertRaises(ValueError):
                ssh_profiles.save_profile(profile)

        with tempfile.TemporaryDirectory() as tmp:
            bad_json = Path(tmp) / "broken.json"
            bad_json.write_text('{"version": 1,', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "JSON 格式无效"):
                ssh_profiles.load_profile(bad_json)
            bad_schema = Path(tmp) / "bad-schema.json"
            bad_schema.write_text(json.dumps({"version": 2}), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "版本"):
                ssh_profiles.load_profile(bad_schema)


if __name__ == "__main__":
    unittest.main()
