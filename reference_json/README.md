# ACE 相机参考模板

把 ACE 相机配置的 JSON/JSONC 参考文件放在这里。默认使用 `uav.json`；
其他单相机模板可通过 `./run_gui.sh --ace-template ./reference_json/uav_12mm.json` 选择。
也可以设置 `ACE_TEMPLATE`。

`uav.json` 是从 `/home/cly/ACE27/config/cameras/uav.json` 复制的参考样板。
导出会保留它的外参、采集参数、注释与字段顺序，只替换标定得到的内参字段。
请根据目标相机维护参考文件。程序不会覆盖参考模板。

## 直接复制参考文件

目前以 uav 为默认参考。在项目根目录执行下面的命令，就可以把 ACE27 的
当前相机配置复制为默认模板（只更新本项目副本，不修改 ACE27 原文件）：

```bash
cp /home/cly/ACE27/config/cameras/uav.json ./reference_json/uav.json
./run_gui.sh
```

也可以在文件管理器中复制一份相机配置到本目录，命名为 `uav.json`。
若保留其他文件名，启动时用 `--ace-template` 指定，例如：

```bash
./run_gui.sh --ace-template ./reference_json/uav_12mm.json
```

参考文件是完整相机配置，不是标定结果的 `camera_info.yaml`。保留
`schema_version: 1` 为首键，以及 `cameras.camera_0` 等相机节点的原结构。
当前默认 uav 是单相机文件；多相机文件（例如 hero、sentry）按现有导出逻辑
只替换第一个匹配相机的内参，不能用它给其他节点标定，请使用单相机参考文件。
导出不会测量安装外参，也不会调整曝光、增益；这些值均继承所复制的参考文件。

2026-10-13 已同步 ACE27 的 uav 最新字段说明；与旧参考文件的配置值完全一致。

模板缺失时，启动日志会提示，仍可正常标定及保存 YAML；点击导出 ACE JSON
才会弹出缺失模板的错误。仓库的 `tests/fixtures/ace_camera.json` 仅供自检，
不会作为 GUI 的自动备用模板。
