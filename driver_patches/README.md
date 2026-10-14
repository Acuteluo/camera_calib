# 工业相机驱动兼容修复

基线：`84dc6ee`（标定加速、设备信息报告）；本轮不改变标定求解算法。

## 使用和恢复

```bash
/usr/bin/python3 -s build_camera_drivers.py --check hik galaxy mv
/usr/bin/python3 -s build_camera_drivers.py hik galaxy mv
./run_gui.sh --driver hik
```

要求原 `$CAMERA_ROOT` 中有三厂完整驱动源码和 SDK，以及已安装 ROS 2/colcon。
`--check` 只读检查源码/SDK架构、补丁适用性、构建工具及 ROS 开发包，
缺项集中显示；OpenCV 和 ROS 2 按已具备处理，不增加安装步骤。
非 Humble 发行版须先 source 对应 ROS setup.bash，使 ROS_DISTRO 指向该发行版。
脚本只复制源码与 SDK，在 `.driver_runtime/` 独立编译，编译失败不切换已启用版本。
成功后原子切换各驱动链接；构建和启动不改写原工作空间、已打开的窗口或相机配置。
原有【写入驱动】和 CLI `install` 仍按用户操作备份并保存原工作空间的内参配置，
新驱动通过显式 `camera_info_url` 读取它，避免新旧版本使用不同的内参文件。
每次构建保留独立版本，不覆盖已运行进程的动态库。

默认海康优先完整 `/opt/MVS`，否则用随包 `hikSDK`；大恒用随包 `GalaxySDK`，
迈德威视用随包 `mvsdk`。可在编译前设置完整 SDK 根目录：
`HIK_SDK_ROOT` / `GALAXY_SDK_ROOT` / `MV_SDK_ROOT`。
SDK 根目录须有 `include` 和匹配架构的 `lib/amd64` 或 `lib/arm64`；MVS 也支持 `lib/64`。
这三个入口指定的是**整套 SDK**，不是某个库文件。

GUI 和 `ros2_calibrate.sh` 都先载入原 ROS 工作空间，再载入修复版 overlay，
且把该 overlay 安装的 SDK 库置于首位。若补丁与已编译版本不一致，要求重建。
在首次成功打开设备时记录型号、序列号和接口；报告优先使用实际打开的设备，
USB VID:PID 仅在 sysfs 中找到同一序列号时补充。

一台匹配相机自动打开；多台同厂相机必须明确选择，防止标错相机：

```bash
CALIB_CAMERA_SERIAL=实际序列号 ./run_gui.sh --driver hik
CALIB_CAMERA_SERIAL=实际序列号 ./ros2_calibrate.sh run galaxy
```

回退：停用相应 `.driver_runtime/hik`、`galaxy` 或 `mv` 链接，再启动即使用原驱动。
也可把链接指向保留的较早构建目录。无需删除正在运行的版本目录。

## 修复范围

| 厂商 | 枚举和选机 | 图像和运行库 |
|---|---|---|
| 海康 MVS | USB + GigE；检查枚举/打开/启动错误；单台自动、多台按 SN | 同一 SDK 头文件与全套运行库；按帧尺寸先分配缓冲区，再由 SDK 转 RGB8；检查转换结果；GigE 最优包长；关闭外触发 |
| 大恒 Galaxy | 全网段枚举 USB/GigE；按 SN 唯一打开 | 修复缓冲区与参数回调的成功判断；Mono8/8-bit Bayer；采集前优先选可用 8-bit 格式；连续采集、关闭外触发；安装同一套 SDK 库 |
| 迈德威视 MindVision | SDK 接口枚举，不固定某型号；单台自动、多台按 SN | 保护初始化失败的句柄；按传感器设置 Mono8/RGB8；先分配图像区再转换；关闭外触发；安装匹配构建的 SDK 库 |

USB3 接线、供电和厂商 udev 权限必须正常；GigE 要设置兼容网段与网卡权限，
跨网段枚举不等于跨网段取流。这里处理面阵单目图像，不包括线阵、采集卡、RTSP 或 UVC。
某型号不支持需要的 SDK 节点或输出格式时会报具体错误，不能根据厂商名断言全部型号已验证。

## 本轮验证

本机海康 MVS 发布说明：`/opt/MVS/ReleaseNote_CH.txt` 标注工业相机 SDK 4.8.2.2；
SDK 实际 `MV_CC_GetSDKVersion()` 返回 `0x04080202`。
USB、GigE 枚举调用返回成功，目前均为 0 台。进程映射确认控制库、USB/GigE 传输库及
格式转换库均来自独立 overlay 中的同一 SDK 套件。

GUI 启动修复版后读取设备当前曝光/增益及参数范围，保留设备当前值；
迈德威视模拟增益按整数发送，避免 ROS 参数类型不匹配。

三厂最终编译、库解析和最小回归结果见 `验证记录/2026-10-14_SDK兼容.md`。
实际 USB/GigE 取图尚需相机在场验证；本轮没有停止或重启已有窗口。

## 原始归档部署验证

从本机 `camera.zip` 重新解压驱动，排除原 `build/install/log`，三厂预检与实际构建通过。
迈德威视归档版与本机源码仅有注释/空格差异，提供 `mv_archive.patch`；脚本自动选择
匹配的已支持补丁。两版补丁后的源码/CMake/配置逐字节一致。
该厂任一补丁更新都会要求重建，避免实际运行的旧补丁与仓库状态不一致。
这验证源码从零构建流程，未在独立新系统或新相机上完成实机取流验收。
