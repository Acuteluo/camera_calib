# 三个相机驱动（海康 / 大恒 / 迈德威视）标定实操指南

> 针对 `$CAMERA_ROOT` 里这三个 ROS 2 驱动。下面每一条"实测"结论都在本机跑过；
> 唯一没条件跑的是"相机插上后标定 GUI 的全流程"（当前没有工业相机接入）。

配套脚本：`ros2_calibrate.sh`（自动探测话题、起标定、回写结果）。
海康的逐条命令启动顺序（含预期输出与故障对照表）：见 `海康标定全流程.md`。

---

## 1. 三个驱动的实测状态

| 驱动目录 | ROS 包 | 相机 | SDK 实测 | 结论 |
|---|---|---|---|---|
| `ros2-hik-camera-main` | `hik_camera` | 海康机器人 USB3（本机 udev 规则显示是 **MV-CS016-10UC**） | ✅ `Found camera count = 0`（SDK 正常，仅缺相机） | **可直接用** |
| `rm_vision_ros2_galaxy_camera-master` | `galaxy_camera` | 大恒 Galaxy USB3 | ✅ `No camera found. device_count = 0`（两个版本 SDK 都正常） | **可直接用**，见 §5 |
| `ros2_mindvision_camera` | `mindvision_camera` | 迈德威视 USB | ✅ `Found camera count = 0` | 可直接用，udev 规则有瑕疵，见 §6 |

三个都已经 `colcon build` 过，`install/` 里的 `lib*.so` 都是 **x86-64**（不是 arm64 那份），架构没问题。

**已顺手修掉的一个坑**：`ros2-hik-camera-main/install/hik_camera/lib/hik_camera/hik_camera_node`
原本权限是 `-rw-rw-r--`，**丢了可执行位**，所以 `ros2 pkg executables hik_camera` 是空的、
`ros2 launch hik_camera ...` 会直接失败。已 `chmod +x` 修好。如果以后重新解压这个包，记得再查一次：

```bash
ls -l $CAMERA_ROOT/ros2-hik-camera-main/install/hik_camera/lib/hik_camera/hik_camera_node
# 需要是 -rwxr-xr-x
```

---

## 2. 三个驱动的共同设计（决定了标定怎么接）

读源码 + 实测确认，三者几乎同构：

1. **话题**：用 `image_transport::create_camera_publisher(this, "image_raw", qos)` 发布，
   编码一律 `rgb8`。image_transport 的相机发布器会同时发 `image_raw` 和它同命名空间的
   `camera_info` 话题。
   → **所以标定只需要订阅 `/image_raw`**（`cameracalibrator` 单目模式也只订阅图像）。
2. **只在相机打开成功后才创建话题和服务**。源码里 `if (i_camera_counts == 0) return;`
   （大恒/迈德威视）和 `while` 枚举循环（海康）。没插相机时 `ros2 topic list` 里
   **一个话题都看不到**，这是正常的，不是驱动坏了。
3. **内参从 `config/camera_info.yaml` 加载**（`camera_info_manager`），所以**标定结果直接写回这个文件即可**，
   重启驱动就会生效。
4. **没有暴露宽高/ROI 参数**（海康只声明 `exposure_time`/`gain`；迈德威视多几个增益/伽马参数）。
   分辨率是**相机内部当前设置**决定的。
   → ⚠️ **`config/camera_info.yaml` 里的 `image_width/height` 必须与相机实际输出一致**。
   本机现状：海康配置写的是 1440×1080，大恒/迈德威视写的是 1280×1024；插上相机后先确认实际分辨率再标。
5. `camera_name` 参数（决定 `set_camera_info` 服务名前缀）在海康/大恒是 `narrow_stereo`，
   在迈德威视是 `mv_camera`；`use_sensor_data_qos` 默认 `false`（reliable QoS，标定程序能自适应）。

---

## 3. 一键脚本用法

```bash
cd ~/camera_calib

./ros2_calibrate.sh list                 # 三个驱动的路径与编译状态
./ros2_calibrate.sh check hik            # 检查环境 + 启动节点 6 秒看 SDK/相机是否就绪
./ros2_calibrate.sh check galaxy
./ros2_calibrate.sh check mv

# 全流程：起驱动 -> 自动探测 /image_raw -> 起标定窗口 -> 结束后回写 camera_info.yaml
./ros2_calibrate.sh run hik --size 8x6 --square 0.0247

# 只回写结果（cameracalibrator 的 SAVE 产物）
./ros2_calibrate.sh install hik /tmp/calibrationdata.tar.gz
```

`run` 干了什么：
1. 校验 `camera_calibration` 是否安装（没装会直接给出 apt 命令）；
2. 后台起驱动，轮询等 `/image_raw` 出现（**自动探测真实话题名，不靠猜**）；
3. 用驱动配置里的 `camera_name` 起 `cameracalibrator`；
4. 你点完 SAVE 关掉窗口后，自动从 `/tmp/calibrationdata.tar.gz` 解出 `ost.yaml`，
   备份原 `config/camera_info.yaml`（`.bak.<时间戳>`）并写回，同时放一份到 `~/.ros/camera_info/`。

`run` 的实测情况：**前置检查与"未装标定包"分支已验证**；等话题→GUI 这一段需要相机在场才能跑完。

---

## 4. 海康（你的相机）完整步骤

```bash
# 0) 装标定程序（一次性）
sudo apt install -y ros-humble-camera-calibration ros-humble-image-pipeline ros-humble-cv-bridge

# 1) 插上相机（USB3 口，直连不要过 hub），确认设备出现
lsusb | grep -i 2bdf          # Hikrobot 的 VID 是 2bdf

# 2) 检查（这一步会告诉你 SDK 好不好、相机在不在）
cd ~/camera_calib && ./ros2_calibrate.sh check hik

# 3) 标定：内角点 8x6，方格边长用卡尺实测值（示例 24.7mm）
./ros2_calibrate.sh run hik --size 8x6 --square 0.0247
```

如果不想用脚本，等价的手工命令是：

```bash
source /opt/ros/humble/setup.bash
source $CAMERA_ROOT/ros2-hik-camera-main/install/setup.bash
ros2 launch hik_camera hik_camera.launch.py &          # 起了之后看 ros2 topic list
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.0247 -c narrow_stereo --no-service-check \
  --ros-args -r image:=/image_raw -r camera:=/narrow_stereo
```

关于 `-r camera:=/narrow_stereo`：驱动的 `camera_info_manager` 会提供
`<camera_name>/set_camera_info` 服务（`camera_name` 来自 `camera_params.yaml`），
这个服务**只在相机打开后**才存在。标定窗口里的 **COMMIT** 按钮就是调它。
不过 COMMIT 只有在驱动用 `file://` 形式的 `camera_info_url` 启动时才会把 YAML 落盘，
所以最稳的做法是**不依赖 COMMIT**，直接用 `install` 子命令写回 `config/camera_info.yaml`，
或者给 launch 传一个可写路径：

```bash
mkdir -p ~/.ros/camera_info
ros2 launch hik_camera hik_camera.launch.py \
  camera_info_url:=file://$HOME/.ros/camera_info/hik.yaml
```

标定后验收：

```bash
# 回写后的内参
grep -A4 camera_matrix $CAMERA_ROOT/ros2-hik-camera-main/config/camera_info.yaml
# 用标定结果去畸变看效果（先用 MVS 或驱动存一张图）
python3 ~/camera_calib/undistort_with_yaml.py \
  --yaml ~/.ros/camera_info/hik.yaml --image test.png --output check.jpg
```

> **MV-CS016-10UC 是 1600 万像素相机**（原始 5328×3032），而现有配置写的是 1440×1080。
> 插上后先用 `ros2 topic echo /image_raw --once --field width`（或看驱动日志）确认实际输出分辨率，
> 因为**标定与分辨率强绑定**：分辨率对不上，标定结果就不可用。
> 需要改分辨率/ROI/binning 时，MVS GUI（`/opt/MVS/bin/MVS`）最方便——但本机**没装 MVS**，
> 而这份驱动自带 `hikSDK`，所以不装 MVS 也能取流标定；只是改相机内部参数需要 MVS 或写一小段 SDK 脚本。

---

## 5. 大恒（galaxy_camera）：**实测正常**（重要更正）

### 5.1 结论

**大恒驱动是好的，可以直接用。** 本机有两种 SDK，实测都能正常初始化并枚举设备：

```
A) 系统版 /usr/lib/libgxiapi.so      (2025-07)  ->  No camera found. device_count = 0   ✅
B) 自带版 GalaxySDK/lib/amd64/...    (2024-09)  ->  No camera found. device_count = 0   ✅
```

> **更正说明**：本文早期版本写过"大恒 SDK 起不来（`fffffc17` / log4cplus 报错）"，
> 那是**误判**——当时是在受限沙箱里跑的，沙箱把工作区外的路径挂成只读，
> 而大恒 SDK 初始化时**要写自己的日志** `/var/log/Galaxy/SDK.log`，写不进去就报错退出。
> 给足文件权限后两个版本都一次通过。**这不是 SDK 的问题，也不是相机的问题。**

### 5.2 唯一的硬性要求：`/var/log/Galaxy` 必须可写

大恒 SDK 的日志配置在 `/etc/Galaxy/cfg/log4cplus.properties`（安装 SDK 时生成），
里面写着 `File=/var/log/Galaxy/SDK.log`。该目录不存在或不可写时，`GXInitLib()` 会直接失败。

本机现状（已满足）：

```bash
$ ls -ld /var/log/Galaxy
drwxrwxrwx 2 nobody nogroup 4096  /var/log/Galaxy     # 0777，谁都能写
```

**换机器/换账号后如果大恒起不来**，先跑这一条：

```bash
sudo mkdir -p /var/log/Galaxy && sudo chmod 777 /var/log/Galaxy
```

图形界面启动大恒前会**自动检查**这个目录，不可写时会直接在日志里打印上面这条命令。

### 5.3 两个版本选哪个

用哪个都行，驱动默认加载的是 `install/galaxy_camera/lib/libgxiapi.so`（随包自带的 2024-09 版）。
想强制用系统里的新版：

```bash
source /opt/ros/humble/setup.bash
source $CAMERA_ROOT/rm_vision_ros2_galaxy_camera-master/install/setup.bash
LD_LIBRARY_PATH=/usr/lib:$LD_LIBRARY_PATH ros2 launch galaxy_camera galaxy_camera.launch.py
```

> 原则：`libgxiapi.so` 与 `GxGVTL.cti`/`GxU3VTL.cti` 要**同版本配套**，
> 不要只替换其中一个（`/usr/lib` 是 2025-07，自带的是 2024-09）。

## 6. 迈德威视的 udev 规则是坏的

`/etc/udev/rules.d/88-mvusb.rules` 内容被截断了：

```
... ATTR{idVendor}=="f622", MODE="666", TAG="mvusb_dev",  A"
... ATTR{idVendor}=="080b", MODE="666", TAG="mvusb_dev",  A"
... ACTION=="remove", TAG=="mvusb_dev", R"
```

末尾的 `A"` / `R"` 是残片（原文应是 `RUN+="..."`），**udev 会整行解析失败**，导致 `f622`（迈德威视）
设备的 `MODE=666` 不生效，普通用户可能打不开相机。`99-mvusb.rules` 那条反而是好的。
建议把 `88-mvusb.rules` 里那三行删掉或重写为：

```
SUBSYSTEM=="usb", ATTR{idVendor}=="f622", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="080b", MODE="0666"
```

改完执行 `sudo udevadm control --reload-rules && sudo udevadm trigger`。

---

## 7. 权限现状（三个驱动通用）

* `/etc/udev/rules.d/99-galaxy-u3v.rules` 已覆盖：
  * 大恒 `2ba2` → `MODE=0666` + `uaccess`
  * 海康 `2bdf:0001` → `GROUP="plugdev" MODE=0660` + `uaccess`
* 你的用户在 `plugdev` 组里（`/etc/group` 里有 `plugdev:x:46:cly`），
  但当前 shell 的 `id` 还没体现出来 → **加组后没重新登录**。
  桌面会话（xorg）里靠规则里的 `TAG+="uaccess"` 通常就能访问；要彻底干净：

```bash
newgrp plugdev        # 当前 shell 立即生效
# 或注销重新登录
```

* 海康的 `2bdf` 规则限定了 `idProduct=="0001"`。如果你的相机 PID 不是 `0001`，
  用 `lsusb` 看实际 PID，把规则放宽成只匹配 VID：
  `SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", MODE="0666"`

---

## 8. 一句话流程

1. `./ros2_calibrate.sh check <驱动>` —— 先确认 SDK 与相机都在位（没插相机会明确告诉你是"没相机"还是"SDK 坏"）。
2. 确认相机实际输出分辨率 == `config/camera_info.yaml` 里的 `image_width/height`。
3. `./ros2_calibrate.sh run <驱动> --size <内角点> --square <实测边长(米)>`。
4. GUI 里把 X / Y / Size / Skew 四条填绿 → CALIBRATE（RMS < 0.5 px）→ SAVE。
5. 脚本自动回写 `config/camera_info.yaml`（带备份），重启驱动生效；再用
   `undistort_with_yaml.py` 目视验收。
