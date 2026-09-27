#!/usr/bin/env bash
# =============================================================================
# 安装/修复相机 udev 规则（需要 root，请用 sudo 运行）
#
#   sudo ./install_udev.sh                    # 安装到 /etc/udev/rules.d 并重载
#   ./install_udev.sh /tmp/某个目录            # 只测流程（不需要 root）
#
# 修的三件事：
#   1. 88-mvusb.rules 被截断损坏（行尾是 A" / R"），整行解析失败 → 重写
#   2. 99-mvusb.rules 引用了不存在的用户组 mvusb_dev → 改成 MODE=0666
#   3. 海康 2bdf 原本锁成 0660 + plugdev（要重新登录才生效）→ 放宽为 0666
# 每个被覆盖的文件都会先备份成 <原名>.bak.<时间戳>
# =============================================================================
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIR="${1:-/etc/udev/rules.d}"
FILES=(88-mvusb.rules 99-mvusb.rules 99-galaxy-u3v.rules)

if [[ ! -d "$DIR" ]]; then
  echo "错误：目录不存在 $DIR" >&2
  exit 1
fi

if [[ "$DIR" == "/etc/udev/rules.d" && "$(id -u)" != "0" ]]; then
  echo "错误：安装到 $DIR 需要 root，请用： sudo $0" >&2
  exit 1
fi

TS="$(date +%Y%m%d_%H%M%S)"
for f in "${FILES[@]}"; do
  [[ -f "$HERE/$f" ]] || { echo "错误：找不到源文件 $HERE/$f" >&2; exit 1; }
  if [[ -f "$DIR/$f" ]]; then
    cp -a "$DIR/$f" "$DIR/$f.bak.$TS"
    echo "已备份  $DIR/$f  ->  $f.bak.$TS"
  fi
  install -m 644 "$HERE/$f" "$DIR/$f"
  echo "已安装  $DIR/$f"
done

if [[ "$DIR" == "/etc/udev/rules.d" ]]; then
  udevadm control --reload-rules && echo "udev 规则已重载"
  udevadm trigger --subsystem-match=usb && echo "已触发 usb 子系统重新应用规则"
  echo
  echo "完成。把相机拔了重插（或重新登录）后生效。"
  echo "验证：lsusb 找到相机后执行   ls -l /dev/bus/usb/*/*  应能看到该设备权限为 crw-rw-rw-"
else
  echo
  echo "（目标不是 /etc/udev/rules.d，只做了文件安装，未重载 udev）"
fi
