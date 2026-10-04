# ACE 相机参考模板

把 ACE 相机配置的 JSON/JSONC 参考文件放在这里。默认使用 `uav.json`；
其他模板可通过 `./run_gui.sh --ace-template ./reference_json/hero.json` 选择。
也可以设置 `ACE_TEMPLATE`。

`uav.json` 是从 `/home/cly/ACE27/config/cameras/uav.json` 复制的参考样板。
导出会保留它的外参、采集参数、注释与字段顺序，只替换标定得到的内参字段。
请根据目标相机维护参考文件。程序不会覆盖参考模板。

模板缺失时，启动日志会提示，仍可正常标定及保存 YAML；点击导出 ACE JSON
才会弹出缺失模板的错误。仓库的 `tests/fixtures/ace_camera.json` 仅供自检，
不会作为 GUI 的自动备用模板。
