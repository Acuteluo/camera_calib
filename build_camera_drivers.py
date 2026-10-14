#!/usr/bin/env python3
"""在独立目录应用已核对的驱动补丁；原工作空间和运行中的节点不变。"""
import argparse
import os
import platform
import sys
from pathlib import Path
import shutil
import subprocess
import tempfile

from driver_runtime import HERE, build_drivers, default_camera_root, driver_patch_digest

DRIVERS = build_drivers(default_camera_root())

ROOT = Path(HERE) / '.driver_runtime'


def matching_patch(driver, source):
    candidates = [Path(HERE) / 'driver_patches' / (driver + '.patch')]
    candidates += sorted((Path(HERE) / 'driver_patches').glob(driver + '_*.patch'))
    messages = []
    for candidate in candidates:
        if not candidate.is_file():
            continue
        result = subprocess.run(['patch', '--dry-run', '--batch', '-p1', '-i', str(candidate)],
                                cwd=source, capture_output=True, text=True)
        if result.returncode == 0:
            return candidate
        messages.append(result.stdout + result.stderr)
    raise RuntimeError('%s 原始源码与已支持补丁不匹配：\n%s' % (driver, '\n'.join(messages)))


def check(drivers):
    """只读预检；缺项全部列出，不启动相机、不编译、不建立结果目录。"""
    errors = []
    for tool in ('patch', 'colcon', 'cmake', 'g++'):
        if not shutil.which(tool):
            errors.append('缺少 %s；请准备驱动构建工具' % tool)
    ros = os.environ.get('ROS_DISTRO', 'humble')
    setup = Path('/opt/ros') / ros / 'setup.bash'
    if not setup.is_file():
        errors.append('缺少 ROS 2 环境：%s；先安装 ROS 2' % setup)
    else:
        for package in ('ament_cmake', 'ament_cmake_auto', 'rclcpp', 'rclcpp_components',
                        'sensor_msgs', 'camera_info_manager', 'image_transport', 'image_transport_plugins'):
            if not (setup.parent / 'share' / package / 'package.xml').is_file():
                errors.append('缺少 ROS 开发包 %s；请准备对应 ROS 驱动开发包' % package)
    machine = platform.machine()
    architecture = {'x86_64': ('amd64', 62), 'aarch64': ('arm64', 183)}.get(machine)
    if not architecture:
        errors.append('不支持的主机架构：' + machine)
    for driver in drivers:
        source = Path(DRIVERS[driver]['ws'])
        for filename in ('package.xml', 'CMakeLists.txt', 'config/camera_info.yaml'):
            if not (source / filename).is_file():
                errors.append('%s 缺少 %s；核对 CAMERA_ROOT=%s' % (driver, filename, source.parent))
        sdk_name, variable, header, library = {
            'hik': ('hikSDK', 'HIK_SDK_ROOT', 'MvCameraControl.h', 'libMvCameraControl.so'),
            'galaxy': ('GalaxySDK', 'GALAXY_SDK_ROOT', 'GxIAPI.h', 'libgxiapi.so'),
            'mv': ('mvsdk', 'MV_SDK_ROOT', 'CameraApi.h', 'libMVSDK.so'),
        }[driver]
        default = Path('/opt/MVS') if driver == 'hik' and Path('/opt/MVS/include/MvCameraControl.h').is_file() else source / sdk_name
        sdk = Path(os.environ.get(variable) or default).expanduser().resolve()
        if not (sdk / 'include' / header).is_file():
            errors.append('%s SDK 缺少 include/%s：%s（可设置 %s）' % (driver, header, sdk, variable))
        if architecture:
            arch, elf_machine = architecture
            libdir = sdk / 'lib' / arch
            if driver == 'hik' and not (libdir / library).is_file():
                libdir = sdk / 'lib' / '64'
            shared = libdir / library
            if not shared.is_file():
                errors.append('%s SDK 缺少当前架构运行库：%s' % (driver, shared))
            else:
                with shared.open('rb') as stream:
                    elf = stream.read(20)
                endian = 'little' if len(elf) > 5 and elf[5] == 1 else 'big'
                if elf[:4] != b'\x7fELF' or len(elf) < 20 or int.from_bytes(elf[18:20], endian) != elf_machine:
                    errors.append('%s SDK 运行库架构不匹配：%s' % (driver, shared))
        if (source / 'CMakeLists.txt').is_file() and shutil.which('patch'):
            try:
                matching_patch(driver, source)
            except RuntimeError as error:
                errors.append(str(error))
        print('%s：源码 %s；SDK %s' % (driver, source, sdk))
    for error in errors:
        print('错误：' + error, file=sys.stderr)
    if not errors:
        print('驱动构建前置检查通过；未启动相机。')
    return not errors


def build(driver):
    original = Path(DRIVERS[driver]['ws'])
    patch = matching_patch(driver, original)
    patch_hash = driver_patch_digest(driver)
    if not (original / 'package.xml').is_file():
        raise RuntimeError('找不到驱动源码：%s' % original)
    ROOT.mkdir(exist_ok=True)
    # 每次编译独立新目录；发布只切换链接，不覆盖正在使用的库。
    destination = Path(tempfile.mkdtemp(prefix=driver + '-', dir=ROOT))
    patch_bytes = patch.read_bytes()
    patch = destination / 'driver.patch'
    patch.write_bytes(patch_bytes)
    source = destination / 'src' / DRIVERS[driver]['pkg']
    project_files = {'CMakeLists.txt', 'package.xml', 'src', 'include', 'config', 'launch',
                     'hikSDK', 'GalaxySDK', 'mvsdk', 'resource', 'LICENSE'}
    def ignore(directory, names):
        if Path(directory) == original:
            return set(names) - project_files
        return {'build', 'install', 'log', '.git'}.intersection(names)
    shutil.copytree(original, source, ignore=ignore, ignore_dangling_symlinks=True)
    dry = subprocess.run(['patch', '--dry-run', '--batch', '-p1', '-i', str(patch)],
                         cwd=source, capture_output=True, text=True)
    if dry.returncode:
        raise RuntimeError('源码与补丁不匹配，未发布新驱动：\n' + dry.stdout + dry.stderr)
    subprocess.run(['patch', '--batch', '-p1', '-i', str(patch)], cwd=source, check=True)
    ros = os.environ.get('ROS_DISTRO', 'humble')
    setup = Path('/opt/ros') / ros / 'setup.bash'
    if not setup.is_file():
        raise RuntimeError('找不到 ROS 环境：%s' % setup)
    env = dict(os.environ, PYTHONNOUSERSITE='1')
    env.pop('PYTHONPATH', None)
    env.pop('PYTHONHOME', None)
    env['PATH'] = '/usr/bin:/bin:' + env.get('PATH', '')
    subprocess.run(['bash', '-c', 'source "$1" || exit $?; exec colcon build '
                    '--cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release',
                    'build-driver', str(setup)], cwd=destination, env=env, check=True)
    (destination / 'patch.sha256').write_text(patch_hash + '\n')
    # 只有成功编译后才启用；旧版本留在目录中，可供已有进程继续运行或回退。
    link = ROOT / (driver + '.new')
    if link.is_symlink():
        link.unlink()
    link.symlink_to(destination.name)
    link.replace(ROOT / driver)
    print('已启用 %s：%s（只影响下一次启动驱动）' % (driver, destination))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='只检查源码、SDK、架构与构建依赖')
    parser.add_argument('drivers', nargs='+', choices=list(DRIVERS))
    args = parser.parse_args()
    try:
        if not check(args.drivers):
            sys.exit(1)
        if not args.check:
            for name in args.drivers:
                build(name)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print('错误：%s；未发布失败的构建，已有驱动仍可用。' % error, file=sys.stderr)
        sys.exit(1)
