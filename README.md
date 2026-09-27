# 棋盘格相机标定实战（海康相机 · Ubuntu 22.04 + ROS 2 Humble）

> 本文所有命令、参数、输出格式都**实测验证过**：ROS 2 `camera_calibration`
> 包自带的标定核心已用合成数据跑通，本仓库自检全部通过（见文末）。

---

## 许可证

MIT License，见 [LICENSE](LICENSE)。Copyright (c) 2026 Acuteluo。

---

## 环境要求

### 必需

| 项目 | 要求 | 说明 |
|---|---|---|
| 系统 | **Ubuntu 22.04（jammy）** | 其它发行版/版本未验证 |
| Python | 3.10（系统自带） | |
| ROS 2 | **Humble**（`ros-humble-desktop`） | 提供 `rclpy` / `sensor_msgs` / `image_transport` / `cv_bridge` |
| 系统包 | `python3-numpy` `python3-opencv` `python3-yaml`<br>`python3-pil` `python3-pil.imagetk` `python3-tk` | ROS desktop 已带 numpy / opencv / yaml / tkinter；**PIL 要自己装**，且 Ubuntu 上 `python3-pil` 与 `python3-pil.imagetk` 是两个包（GUI 的 `ImageTk` 在后者里） |
| 中文字体 | `fonts-noto-cjk` | 缺它界面中文会显示成方框；程序仍能运行 |

**一条命令装齐**（只装系统包，不动你的代码和标定结果）：

```bash
./install_deps.sh            # 检查并安装缺失的依赖
./install_deps.sh --check    # 只检查、只报告，不需要 sudo
```

### 可选

| 你要用的功能 | 额外需要 |
|---|---|
| `./ros2_calibrate.sh`（ROS 官方 cameracalibrator 编排） | `ros-humble-camera-calibration`、`ros-humble-image-pipeline`（**不在 desktop 里**，需单独装） |
| 图形界面的【启动驱动】按钮 | 三个相机驱动的工作空间，见下方 `$CAMERA_ROOT` |
| 【导出 ACE json】/ 输出 ACE 配置 | 一份 ACE 相机配置模板（默认找 `~/ACE27/config/cameras/uav.json`，可用 `ACE_TEMPLATE` 或 `--ace-template` 指定） |
| 纯 OpenCV 路线（`calibrate_mono.py` / `capture_chessboard.py` / `selftest_synthetic.py`） | **完全不需要 ROS**，只要 numpy + opencv（Pillow 仅 `capture_chessboard.py` 叠中文时用） |

### 版本兼容性（已实测）

| 依赖 | 实测版本 | 结论 |
|---|---|---|
| OpenCV（**apt**，ROS 自带） | **4.5.4** | ✔ GUI 启动 + 全链路自检通过 |
| OpenCV（pip） | 4.9.0 / 4.11.0 | ✔ 通过 |
| numpy | 1.21.5（apt）/ 1.26.4（pip） | ✔ |
| PyYAML | 5.4.1 | ✔ |
| Pillow | 9.0.1 | ✔ |
| ROS 2 | Humble | ✔ |

> 用到的 OpenCV API（`findChessboardCornersSB`、`CALIB_CB_ACCURACY`、`CALIB_CB_EXHAUSTIVE`、
> `cornerSubPix`、`getOptimalNewCameraMatrix`、`undistortPoints`、`aruco`）在 **4.5.4 起全部可用**，
> 所以 ROS 2 Humble 自带的 OpenCV 版本足够，**不必额外 pip 装 OpenCV**。

---

## 快速开始

```bash
git clone <仓库地址> ~/camera_calib && cd ~/camera_calib
./install_deps.sh              # 装依赖（ROS 2 需你事先装好）

# 自检：不需要相机，约 30 秒跑完
python3 selftest_synthetic.py  # 纯 OpenCV 全链路（合成图反解内参，13 项断言）
python3 calib_gui.py --self-test   # 界面逻辑 + ACE 配置渲染
```

三条使用路线，按需选一条：

| 路线 | 入口 | 需要 ROS 2？ |
|---|---|---|
| **图形界面（推荐）** | `./run_gui.sh` | 需要 |
| **纯 OpenCV 命令行** | `python3 calibrate_mono.py --images ./images --size 11x8 --square 0.0247` | **不需要** |
| **ROS 2 官方编排** | `./ros2_calibrate.sh run hik` | 需要 |

### 关于 `$CAMERA_ROOT`

文档里的 `$CAMERA_ROOT` 指**三个相机驱动工作空间的根目录**，程序默认自动探测
`~/下载/camera`、`~/Downloads/camera`、`~/camera`、`~/camera_ws`、`/opt/camera`
这些常见位置；换到别处**不用改源码**，设环境变量即可：

```bash
export CAMERA_ROOT=/path/to/camera_ws        # 三个驱动工作空间所在根目录
export CALIB_RESULTS_DIR=/path/to/results    # 标定结果归档目录（默认 <工具目录>/标定结果）
export ACE_TEMPLATE=/path/to/camera.json     # ACE 相机配置模板（可选，见 §7）
```

三个驱动的子目录名固定为上游仓库名：`ros2-hik-camera-main`、
`rm_vision_ros2_galaxy_camera-master`、`ros2_mindvision_camera`。
程序里也有对应命令行开关：`--camera-root` / `--results-dir` / `--ace-template`。

---

## 0. 先回答你的三个问题

**Q1：能用 ROS 2 的标定程序吗？**
能。而且你贴的那份 YAML **就是** ROS 2 `camera_calibration` 包里 `cameracalibrator`
的输出格式 —— 连 `camera_name: narrow_stereo` 都是它的默认值（源码里 `-c/--camera_name`
的 default 就是 `narrow_stereo`）。所以拿 ROS 2 标定，输出的就是你要的那个文件。

**Q2：本机环境够吗？**
够，但有几个现状要注意（实测结果）：

| 项目 | 实测状态 | 说明 |
|---|---|---|
| 系统 | Ubuntu 22.04.5 LTS (jammy) | ✅ |
| ROS 2 | Humble 已装在 `/opt/ros/humble` | ✅ 但当前 shell 没 source |
| ROS apt 源 | USTC 镜像 `mirrors.ustc.edu.cn/ros2/ubuntu` | ✅ 有候选版本，能装 |
| `camera_calibration` 等包 | **未安装**（候选 3.0.9） | 需 `sudo apt install` |
| OpenCV | pip 装的是 4.11.0 | ⚠️ 与 apt 的 4.5.4 并存，见 §6 坑位 |
| 海康 SDK | `/opt/MVS` **不存在**，但 `下载/camera/ros2-hik-camera-main` 自带 `hikSDK` 且已编译好 | 实测该驱动的 SDK **能正常枚举**，所以**不必装 MVS** 就能取流标定，详见 `ROS2_三驱动标定指南.md` |
| 相机设备 | 当前**没有** `/dev/video*` | 相机没插，或走的不是 UVC |
| 用户组 | `cly` 不在 `video` 组 | 用 `/dev/video*` 前需处理 |

**Q3：标定前必须先分清是哪种"海康相机"** —— 这决定了完全不同的取流方式：

| 类型 | 典型型号 | 接口 | 取流方式 |
|---|---|---|---|
| **A. 海康机器人工业相机** | MV-CA / MV-CU / MV-CS 系列（USB3 / GigE） | USB3 Vision / GigE Vision，**不是 UVC** | 必须装 **MVS SDK**；插上也不会出现 `/dev/video0` |
| **B. 海康威视网络摄像机** | DS-2CD 系列 IPC | RTSP 网络流 | `rtsp://…/Streaming/Channels/101` |
| **C. 海康威视 USB 摄像头** | 免驱 UVC 摄像头 | UVC | `/dev/video0` + `usb_cam`/`v4l2_camera` |

`lsusb` 里如果看到 `Hikvision`/`HIKROBOT` 且有 `/dev/video0` → C；
只有 `lsusb` 有设备但**没有** `/dev/video*` → 大概率是 A。

---

## 1. 标定板怎么准备（决定成败，比软件重要）

* **棋盘格**：推荐 9×7 或 11×8 个方格（即内角点 **8×6 / 10×7**）。
* **关键概念**：`--size` 数的是**内部黑白交叉点**，不是方格数。
  一块 9×7 方格的棋盘 → `--size 8x6`。
* **`--square` 必须是实测值（单位：米）**：打印后用卡尺/钢尺量 5 个方格的总长再除以 5。
  A4 纸打印的 25 mm 标称值常常实际是 24.6 mm，这个 1.6% 误差会**直接变成焦距的 1.6% 误差**。
* **平整**：裱在硬板上（亚克力/铝板）。纸翘了 → 畸变系数直接跑偏且 RMS 下不来。
* **别用屏幕显示棋盘格**：屏幕有摩尔纹、像素栅格会污染角点亚像素定位。
* 棋盘格要**留白边**（四周至少半个方格），否则 `findChessboardCorners` 容易找不到。
* 相机镜头如果是**变焦/自动对焦**的：标定时把焦距固定、对焦环锁死。变焦后标定作废。

---

## 2. 路线一（推荐）：ROS 2 `camera_calibration`

### 2.1 安装

你的 ROS 源已经是 USTC 镜像，直接装：

```bash
sudo apt update
sudo apt install -y ros-humble-camera-calibration ros-humble-image-pipeline \
                    ros-humble-usb-cam ros-humble-v4l2-camera \
                    ros-humble-image-view ros-humble-cv-bridge
```

> `python3-semver` 等依赖会被自动带上。我验证过：这个包的标定核心在本机 OpenCV 4.11 上运行正常。

### 2.2 起相机节点

**类型 C（UVC USB 摄像头）**：

```bash
sudo usermod -aG video $USER      # 只需一次，之后重新登录
source /opt/ros/humble/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 -p image_width:=1440 -p image_height:=1080 \
  -p framerate:=30.0 -p pixel_format:=mjpeg2rgb
```

或者用 `v4l2_camera`（支持 UVC 控制项，曝光/增益好调）：

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0 \
  -p image_size:="[1440,1080]"
```

**类型 B（RTSP 网络摄像机）**：用 `gscam` 或 `ffmpeg_image_transport`，也可以直接用第 3 节的纯 OpenCV 工具采图（最省事）。

**类型 A（MV 工业相机）**：**本机已经有现成方案，不需要装 MVS**：
`$CAMERA_ROOT/ros2-hik-camera-main`（`hik_camera` 包，自带 `hikSDK`，已编译）实测可正常加载 SDK，
启动后发布 `/image_raw`（`rgb8`）。直接：

```bash
./ros2_calibrate.sh check hik     # 先确认 SDK 与相机
./ros2_calibrate.sh run   hik --size 8x6 --square 0.0247
```

若确实需要 MVS（例如要改相机内部 ROI/binning/用户参数），再从海康机器人官网装 `.deb`
（`sudo dpkg -i MVS-*.deb`）。另外社区也有其他驱动（`hikrobot_camera`、`hik_camera`、`hikvision_ros2_driver`）。

### 2.3 起标定程序

```bash
source /opt/ros/humble/setup.bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  -c my_hik --no-service-check \
  --ros-args -r image:=/image_raw
```

参数说明（均核对过源码）：

| 参数 | 含义 |
|---|---|
| `--size 8x6` | 内角点数（宽×高） |
| `--square 0.025` | 方格实测边长，**米** |
| `-c my_hik` | 写进 YAML 的 `camera_name`（默认 `narrow_stereo`，就是你贴的那个） |
| `--no-service-check` | **强烈建议加**：源码启动时会等 `camera/set_camera_info`、`left_camera/...`、`right_camera/...` 三个服务各 5 秒，等不到就 `rclpy.shutdown()`，GUI 直接卡死 |
| `-r image:=/xxx` | 订阅的图像话题（默认话题名就是 `image`，必须重映射到你的相机话题） |
| `-k N` | 径向畸变系数个数，默认 2（即 k1,k2，且固定 k3=0 → `plumb_bob`）；>3 时启用 `rational_polynomial` 模型 |
| `--fix-principal-point` | 强制主点在图像中心（工业相机镜头光轴正、数据少时可用） |
| `--zero-tangent-dist` | 强制切向畸变 p1=p2=0（镜头装配好时可减少过拟合） |
| `--queue-size` | 图像队列，默认 1 |
| `--approximate 0.02` | 双目时两路时间戳容差 |

> **顺序很重要**：先起相机节点，再起标定程序。标定程序会自动查询发布者的 QoS
> （源码里 `get_topic_qos` 会去读 publisher 的 QoS 再照抄），**如果话题还没有发布者，
> 它会退回 reliable QoS**，而很多相机驱动是 best-effort → 订阅不上、`No publishers available`
> 警告、界面永远没画面。

### 2.4 GUI 怎么操作

弹出的窗口左侧是实时画面 + 检测到的角点，右侧有 4 个进度条：
**X、Y（标定板在画面中的位置）、Size（远近/大小）、Skew（倾斜）** —— 四条都变绿，
`CALIBRATE` 按钮才可用（或攒够 40 张样本也会放行）。

采集动作：
1. 先让标定板在**画面正中央**，前后移动改变 Size；
2. 再挪到画面**四个角、四边中点**（把 X、Y 填满）；
3. 最后加**倾斜/旋转**视角（Skew），左右各歪 20°~45°；
4. 每换一个姿态停稳 0.5 s（避免运动模糊）；
5. 进度条全绿 → 点 **CALIBRATE** → 右侧会打印 RMS 和 fx/fy/cx/cy；
6. 满意就点 **SAVE**，然后点 **COMMIT**（COMMIT 会把结果通过 `set_camera_info`
   服务推给驱动；没重映射 `camera:=` 时这个按钮没用，但不影响 SAVE 的文件）。

### 2.5 结果文件在哪

`SAVE` 写的是：

```
/tmp/calibrationdata.tar.gz
```

解开就是 `ost.yaml`（**就是你要的那个格式**）和一堆采集到的 `*.pgm`：

```bash
mkdir -p ~/calib_out && tar -xzf /tmp/calibrationdata.tar.gz -C ~/calib_out
cat ~/calib_out/ost.yaml
```

要长期用，把它放到 ROS 约定位置并改个好名字：

```bash
mkdir -p ~/.ros/camera_info
cp ~/calib_out/ost.yaml ~/.ros/camera_info/my_hik.yaml
```

重映射服务名（想让 COMMIT 生效时用，驱动节点名要对上）：

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  -c my_hik --ros-args -r image:=/camera/image_raw -r camera:=/camera
```

---

## 3. 路线二：不依赖 ROS 的纯 OpenCV 工具（本目录已备好）

适合：工业相机用 MVS 抓图、网络相机抓 RTSP、或者你只想快速出一份 YAML。

```bash
cd ~/camera_calib

# ① 采图（UVC 相机 / RTSP；按空格保存，a 键自动采集，q 退出）
python3 capture_chessboard.py --device /dev/video0 --size 8x6 \
        --width 1440 --height 1080 --out ./images --auto
# 网络摄像机：
python3 capture_chessboard.py --url "rtsp://admin:pass@192.168.1.64:554/Streaming/Channels/101" \
        --size 8x6 --out ./images

# ② 标定 -> 直接产出与 ROS 完全同格式的 YAML
python3 calibrate_mono.py --images ./images --size 8x6 --square 0.0247 \
        --camera-name my_hik --output my_hik.yaml --k 2 --save-undistorted

# ③ 用标定结果去畸变，肉眼验收
python3 undistort_with_yaml.py --yaml my_hik.yaml --image ./images/img_000.png \
        --output check.jpg
```

`calibrate_mono.py` 会打印：每张图的检测结果、整体 RMS、最差视图、角点覆盖画面比例、
以及超阈值时的警告。支持 `chessboard` / `circles` / `acircles` / `charuco`（ChArUco 更强壮）。

---

## 4. 你贴的那份 YAML，逐字段解读

```yaml
image_width: 1440              # 标定时的图像宽（必须与你实际用图一致！）
image_height: 1080
camera_name: narrow_stereo     # cameracalibrator 的默认相机名
camera_matrix:                 # 内参 K，针孔模型
  rows: 3
  cols: 3
  data: [1750.34699,    0.     ,  733.30737,   # fx, 0, cx
            0.     , 1749.40469,  549.1027 ,   # 0, fy, cy
            0.     ,    0.     ,    1.     ]
distortion_model: plumb_bob    # 5 参数模型 (k1,k2,p1,p2,k3)；>5 参数时是 rational_polynomial
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.056721, 0.044929, -0.000139, -0.000374, 0.000000]
                               # k1, k2, p1, p2, k3
rectification_matrix:          # 单目恒为单位阵；双目才有值
  rows: 3
  cols: 3
  data: [1., 0., 0.,  0., 1., 0.,  0., 0., 1.]
projection_matrix:             # 去畸变后使用的 3x4 投影矩阵 P
  rows: 3
  cols: 4
  data: [1735.62931,    0.     ,  733.16557,    0.,
            0.     , 1740.60373,  549.12198,    0.,
            0.     ,    0.     ,    1.     ,    0.     ]
```

**为什么 `P` 和 `K` 的焦距不一样？**（1750.35 vs 1735.63 —— 很多人以为写错了）
不是笔误。源码里单目标定时：

```python
ncm, _ = cv2.getOptimalNewCameraMatrix(self.intrinsics, self.distortion, self.size, a)  # a = 0.0
self.P[:3, :3] = ncm
```

即 `P` 是 **alpha=0 的最优新内参**（裁掉去畸变后的无效黑边，等效于轻微放大），
所以 `fx/fy/cx/cy` 会和 `K` 有零点几到几个百分点的差别。我用本机的 OpenCV 复现验证过
（见 `selftest_synthetic.py` 里 `projection_matrix 左上 3x3 == getOptimalNewCameraMatrix(...)` 这条断言）。

**怎么用**：做视觉测量/3D 重建用 `K + D`；做图像去畸变显示用 `P`
（`image_proc` 的 `image_rect`、`cv::initUndistortRectifyMap(K, D, R, P[:3,:3], size, ...)`）。

**数值合理性自查**：
* `fx ≈ fy`（比值应在 0.99~1.01）；你这个 1750.35/1749.40 很好。
* `cx ≈ width/2`（733 ≈ 720）、`cy ≈ height/2`（549 ≈ 540）：你这个也很正常。
* `k1` 一般是个负数（桶形畸变，你这个 -0.057 合理）；`k2` 通常比 k1 小一个量级，
  你这里 0.045 偏大，说明畸变模型有点勉强 —— 如果 RMS 不大就问题不大。

---

## 5. 验收标准（别只看"标定成功了"）

| 指标 | 合格线 | 说明 |
|---|---|---|
| RMS 重投影误差 | **< 0.5 px**，优秀 < 0.3 px | `calibrate_mono.py` 会直接打印 |
| 角点覆盖画面比例 | **> 40%** | 覆盖不够时焦距和畸变会互相"补偿"，换个视角就废 |
| fx/fy 比值 | 0.99 ~ 1.01 | 差太多说明像素非方形或标定不稳 |
| cx/cy 与图像中心偏差 | < 5% 图像尺寸 | 偏差离谱通常是覆盖不足/标定板不平 |
| 去畸变后直线 | 目视笔直 | 用 `undistort_with_yaml.py` 拍带直线/门框的场景看 |
| 交叉验证 | 重拍一组新图，仅"评估"不重标 | RMS 应接近，且角点残差无系统性方向 |

**重要**：标定结果与**分辨率强绑定**。1440×1080 标出来的参数，直接拿去 720×540 用是错的
（要先缩放：`fx' = fx * 720/1440`，`cx' = cx * 720/1440`，畸变系数不变）。
分辨率、像素格式、ROI、binning 一变，就要重标。

---

## 6. 常见坑（按踩坑概率排序）

1. **`No publishers available for topic image` / 界面全黑**
   标定程序先于相机节点启动，QoS 退化成 reliable 订不上 best-effort 的相机话题。
   → 先起相机，再起标定；或确认话题名（`ros2 topic list` / `ros2 topic info -v /image_raw`）。
2. **`Waiting for service camera/set_camera_info...` 卡住然后退出**
   源码会等 3 个服务各 5 s。→ 加 `--no-service-check`。
3. **`image_width/image_height` 与实际不符**
   驱动有缩放/ROI/格式转换。→ 标定前 `ros2 topic echo /camera_info --once` 看清分辨率，
   并且标定与使用时**保持完全一致**。
4. **pip 的 OpenCV 4.11 与 apt 的 4.5.4 冲突**
   `camera_calibration` 是 Python 实现、走 `cv2`。混合环境下可能出现
   `cv2` 版本错乱或 GUI 后端（Qt/GTK）问题。
   → 要么确保只走一套；报 `cv2.error`/窗口打不开时先试 `unset PYTHONPATH` 并用 ROS 环境重跑。
5. **运动模糊 / 自动曝光**
   手持标定板移动中截图 → 角点糊。→ 手动曝光、固定增益、ISO 低、每姿态停稳再采。
6. **标定板不平 / 打印翘边**
   典型症状：RMS 0.5~2 px 下不来，畸变系数乱跳。→ 裱板。
7. **`--square` 用标称值而非实测值**
   引入系统性焦距误差，且 RMS 看不出来（RMS 对整体尺度不敏感）。→ 实测。
8. **工业相机(MV 系列)插上却没有 `/dev/video0`**
   它走 USB3 Vision/GigE Vision，不是 UVC —— 这是正常的。用 `hik_camera` 驱动即可
   （`./ros2_calibrate.sh run hik`），本机已自带 SDK 且实测可用。
9. **`/dev/video0: Permission denied`**
   → `sudo usermod -aG video $USER` 后**重新登录**（或临时 `sudo chmod 666 /dev/video0`）。
10. **高分辨率下 UVC 掉帧**
    `YUYV` 在 1440×1080 通常只能跑到 5 fps 左右。→ 用 `MJPG`（`fourcc=MJPG`，
    我的采集脚本默认就是 MJPG）。

---

## 7. 本目录文件与自检

| 文件 | 作用 |
|---|---|
| `install_deps.sh` | **一键装依赖**：`./install_deps.sh` 安装 / `--check` 只检查（开源后第一步就跑它） |
| `run_gui.sh` + `calib_gui.py` | **图形化标定程序**（全中文界面：步骤引导、灰按钮、曝光/增益滑条+输入框、检验报告、导出 ACE 配置） |
| `ace_json.py` | ACE27 相机配置渲染库：以 `ACE27/config/cameras/*.json` 为模板做文本替换，只换内参字段，注释/键序/缩进逐字节保留 |
| `ace_export.py` | **一键把历史标定结果转成 ACE JSON**：`python3 ace_export.py --dry-run` 看计划 → `python3 ace_export.py` 生成（只新增、不覆盖、不删除） |
| `【看这个】一键标定图形化说明.md` | 图形界面版一键标定教程（推荐先看这个） |
| `ros2_calibrate.sh` | 命令行编排标定：`list` / `check` / `run` / `install` / `verify` / `results`（三个驱动通用） |
| `udev_fix/` | 相机 udev 权限规则修复：`sudo ./udev_fix/install_udev.sh`（说明见其中 `说明.md`） |
| `标定结果/` | 所有标定产物按时间归档：`<时间戳>_<驱动>_<尺寸>/`，含 `latest` 软链接，见其中 `说明.txt` |
| `ROS2_三驱动标定指南.md` | 三个驱动的实测状态、SDK 故障诊断与完整标定流程 |
| `海康标定全流程.md` | 插上海康相机后的逐条命令启动顺序、预期输出、故障对照表 |
| `capture_chessboard.py` | V4L2 / RTSP 采图，实时检测角点，空格手动存 / `a` 自动存（带最小位移判据） |
| `snap.py` | 从 ROS 2 图像话题抓帧存 PNG，配合 `calibrate_mono.py --detect-only` 离线验证 `--size` |
| `calibrate_mono.py` | 标定主程序，输出 ROS `camera_info` 同格式 YAML，含质量报告；`--detect-only` 只验尺寸 |
| `undistort_with_yaml.py` | 读 YAML 去畸变（单图 / 实时），验收标定结果 |
| `selftest_synthetic.py` | **自检**：用已知内参渲染带畸变棋盘格 → 反解 → 校验精度与 YAML 格式 |
| `requirements.txt` | pip 路线的依赖清单（apt 路线用 `install_deps.sh` 即可，见「环境要求」） |

自检命令（不需要相机，约 30 秒跑完，结果可复现）：

```bash
python3 ~/camera_calib/selftest_synthetic.py
python3 ~/camera_calib/calib_gui.py --self-test      # 含 ACE JSON 渲染校验
python3 ~/camera_calib/ace_json.py --self-test       # 只校验模板渲染
```

本机实测结果（真值 fx=1750.35, fy=1749.40, cx=733.31, cy=549.10, k1=-0.056721）：

```
整体 RMS 重投影误差: 0.1010 px
内参 fx=1752.103 fy=1751.143 cx=733.903 cy=549.939
   （fx 误差 1.756 px、fy 误差 1.739 px、cx 误差 0.595 px、cy 误差 0.836 px）
畸变 k1=-0.057242  k2=0.048630   （k1 相对误差 0.9%）
角点包络覆盖画面比例: 51.5%
14 项断言：全部通过
```

> fx 与 k1 在标定里是强相关的（近轴畸变和焦距会互相补偿），所以 fx 差 1.8 px 而 RMS 只有
> 0.1 px 属于正常现象；评测一套标定好不好，看 RMS + 覆盖 + 交叉验证，不要单看某一个数。

另外我还把 ROS 2 `camera_calibration` 包（deb 解包）**自带的标定核心**在同一批合成图上跑了一遍，
它输出的 `ost.yaml` 与本脚本格式逐字段一致，`P` 同样满足 `getOptimalNewCameraMatrix(K,D,size,0)`，
恢复的 fx 误差 3.7 px、cx 误差 0.08 px、k1 误差 2.2% —— 说明两条路等价，ROS 2 那条路在你机器上确实可用。

---

## 8. 最短路径总结

1. 分清相机类型（UVC → `/dev/video0`；MV 工业 → MVS SDK 抓图；IPC → RTSP）。
2. 打印/裱平棋盘格，**卡尺量出真实方格边长（米）**，确定内角点数（如 8×6）。
3. 装包：`sudo apt install ros-humble-camera-calibration ros-humble-usb-cam ros-humble-v4l2-camera`
4. 起相机 → 起 `cameracalibrator --size 8x6 --square <实测> --no-service-check -r image:=<你的话题>`。
5. GUI 里把 X / Y / Size / Skew 四条进度条填绿（四角四边 + 前后 + 倾斜），CALIBRATE。
6. RMS < 0.5 px 才 SAVE；文件在 `/tmp/calibrationdata.tar.gz` → 解出 `ost.yaml`。
7. 用 `undistort_with_yaml.py` 拍直线场景目视验收；必要时重拍、重标。
