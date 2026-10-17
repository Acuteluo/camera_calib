"""Offline SSH deployment tests; no SSH server or ROS installation is used."""

import io
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest import mock

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import remote_deploy


def make_camera_root(root, drivers=("hik",)):
    names = {
        "hik": "ros2-hik-camera-main",
        "galaxy": "rm_vision_ros2_galaxy_camera-master",
        "mv": "ros2_mindvision_camera",
    }
    root = Path(root)
    for driver in drivers:
        source = root / names[driver]
        (source / "config").mkdir(parents=True)
        (source / "src").mkdir()
        (source / "package.xml").write_text("<package/>\n")
        (source / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.10)\n")
        (source / "config" / "camera_info.yaml").write_text("camera_name: test\n")
        (source / "src" / "node.cpp").write_text("// driver source\n")
    return root


class RemoteDeployTests(unittest.TestCase):
    def test_archive_contains_only_core_and_selected_driver_payload(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_camera_root(Path(tmp) / "camera")
            sdk = Path(tmp) / "vendor-sdk"
            (sdk / "include").mkdir(parents=True)
            (sdk / "lib" / "amd64").mkdir(parents=True)
            (sdk / "include" / "MvCameraControl.h").write_text("// header\n")
            (sdk / "lib" / "amd64" / "libMvCameraControl.so").write_bytes(b"sdk")
            (root / "ros2-hik-camera-main" / "hikSDK").mkdir()
            (root / "ros2-hik-camera-main" / "build").mkdir()
            (root / "ros2-hik-camera-main" / "build" / "cache.o").write_bytes(b"generated")
            (root / "ros2-hik-camera-main" / "capture.mcap").write_bytes(b"camera data")

            archive, bundled = remote_deploy._create_archive(
                root, ["hik"], {"HIK_SDK_ROOT": str(sdk)})
            try:
                with tarfile.open(fileobj=archive, mode="r:gz") as tar:
                    names = set(tar.getnames())
                self.assertIn("project/remote_deploy.py", names)
                self.assertIn("project/runtime.sh", names)
                self.assertIn("project/driver_patches/hik.patch", names)
                self.assertIn("project/reference_json/uav.json", names)
                self.assertIn(
                    "project/drivers/ros2-hik-camera-main/hikSDK/include/MvCameraControl.h",
                    names,
                )
                self.assertIn(
                    "project/drivers/ros2-hik-camera-main/hikSDK/lib/amd64/libMvCameraControl.so",
                    names,
                )
                self.assertNotIn("project/tests", names)
                self.assertFalse(any("/.git/" in name for name in names))
                self.assertFalse(any("/build/" in name for name in names))
                self.assertFalse(any(name.endswith("capture.mcap") for name in names))
                self.assertEqual(bundled, {"hik": "HIK_SDK_ROOT"})
            finally:
                archive.close()

    def test_failed_ssh_build_is_reported_and_command_publishes_only_after_build(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_camera_root(Path(tmp) / "camera")
            sdk = Path(tmp) / "vendor-sdk"
            (sdk / "include").mkdir(parents=True)
            (sdk / "lib" / "amd64").mkdir(parents=True)
            (sdk / "include" / "MvCameraControl.h").write_text("// header\n")
            (sdk / "lib" / "amd64" / "libMvCameraControl.so").write_bytes(b"sdk")
            messages = []
            with mock.patch.dict(os.environ, {"HIK_SDK_ROOT": str(sdk)}), \
                    mock.patch.object(remote_deploy, "_run_ssh", return_value=1) as run:
                with self.assertRaisesRegex(RuntimeError, "未发布新项目"):
                    remote_deploy.deploy_remote(
                        "camera-box", 22, None, "~/calib dir", "hik", root, messages.append)

            run.assert_called_once()
            command = run.call_args.args[0][-1]
            self.assertIn("bash -c ", command)
            self.assertLess(command.index("mv -T"), command.index("build_camera_drivers.py"))
            self.assertIn("trap cleanup EXIT", command)
            self.assertIn("release_created=0", command)

    def test_remote_directory_shell_metacharacters_remain_path_data(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            marker = home / "injected"
            remote_dir = "~/camera dir; touch " + str(marker)
            release_id = "a" * 32
            command = remote_deploy._remote_deploy_command(
                remote_dir, release_id, ["hik"], {})
            buffer = io.BytesIO()
            with tarfile.open(fileobj=buffer, mode="w:gz") as tar:
                runtime = b"calib_base_env() { :; }; calib_ros_env() { :; }\n"
                build = b"#!/usr/bin/python3\nprint('BUILD_OK')\n"
                for name, payload in (("runtime.sh", runtime),
                                      ("build_camera_drivers.py", build)):
                    info = tarfile.TarInfo("project/" + name)
                    info.size = len(payload)
                    tar.addfile(info, io.BytesIO(payload))
            result = subprocess.run(
                ["bash", "-c", command], input=buffer.getvalue(), capture_output=True,
                text=False, timeout=5, env={**os.environ, "HOME": str(home)}, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr.decode("utf-8", "replace"))
            self.assertFalse(marker.exists())
            self.assertIn(b"BUILD_OK", result.stdout)
            deployed_line = next(
                line for line in result.stdout.decode("utf-8").splitlines()
                if line.startswith("DEPLOYED_PROJECT=")
            )
            deployed = Path(deployed_line.split("=", 1)[1]).parent
            self.assertTrue((deployed / "project" / "runtime.sh").is_file())
            self.assertTrue((deployed / "READY").is_file())

    def test_remote_build_failure_removes_the_unready_release(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            release_id = "b" * 32
            remote_dir = "~/deploy root"
            command = remote_deploy._remote_deploy_command(
                remote_dir, release_id, ["mv"], {})
            buffer = io.BytesIO()
            with tarfile.open(fileobj=buffer, mode="w:gz") as tar:
                runtime = b"calib_base_env() { :; }; calib_ros_env() { :; }\n"
                build = b"raise SystemExit(7)\n"
                for name, payload in (("runtime.sh", runtime),
                                      ("build_camera_drivers.py", build)):
                    info = tarfile.TarInfo("project/" + name)
                    info.size = len(payload)
                    tar.addfile(info, io.BytesIO(payload))
            result = subprocess.run(
                ["bash", "-c", command], input=buffer.getvalue(), capture_output=True,
                text=False, timeout=5, env={**os.environ, "HOME": str(home)}, check=False,
            )
            self.assertEqual(result.returncode, 7)
            release_root = home / "deploy root" / ".ssh_deploy" / release_id
            self.assertFalse(release_root.exists())

    def test_ssh_output_is_forwarded_line_by_line(self):
        messages = []
        with tempfile.TemporaryFile() as archive:
            code = remote_deploy._run_ssh(
                ["bash", "-c", "printf 'checking\\n'; sleep 0.05; printf 'building\\n'"],
                archive, messages.append, timeout=5,
            )
        self.assertEqual(code, 0)
        self.assertEqual(messages, ["checking", "building"])


if __name__ == "__main__":
    unittest.main()
