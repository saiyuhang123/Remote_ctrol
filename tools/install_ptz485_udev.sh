#!/usr/bin/env bash
# 串口 udev 配置（可重复执行）：
#   /dev/imu_usb   <- USB 口 1-9  上的 CH340（IMU）
#   /dev/ptz485    <- USB 口 1-10 上的 CH340（云台 RS485，PELCO-D 2400 8N1 地址 1）
#
# 背景：两个 CH340 都是 1a86:7523 且没有序列号，旧规则只按 VID/PID 匹配，
# 会把云台适配器也当成 IMU。现在按 USB 物理口固定。
# 另一个坑：udev 的 ATTRS/KERNELS 只能引用同一个父设备，混用会永不匹配。
#
# 用法: sudo bash tools/install_ptz485_udev.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_IMU="$HERE/imu_usb.rules"
SRC_PTZ="$HERE/99-ptz485.rules"
DST_IMU=/etc/udev/rules.d/imu_usb.rules
DST_PTZ=/etc/udev/rules.d/99-ptz485.rules

if [[ $EUID -ne 0 ]]; then
    echo "需要 root：sudo bash $0" >&2
    exit 1
fi
for f in "$SRC_IMU" "$SRC_PTZ"; do
    [[ -f "$f" ]] || { echo "找不到规则文件: $f" >&2; exit 1; }
done

# 备份旧的 IMU 规则（内容不同才备份）
if [[ -f "$DST_IMU" ]] && ! cmp -s "$SRC_IMU" "$DST_IMU"; then
    BAK="$DST_IMU.bak.$(date +%Y%m%d_%H%M%S)"
    cp -a "$DST_IMU" "$BAK"
    echo "[IMU] 已备份旧规则 -> $BAK"
fi
install -m 0644 "$SRC_IMU" "$DST_IMU"
install -m 0644 "$SRC_PTZ" "$DST_PTZ"
echo "[IMU] $DST_IMU"
echo "[PTZ] $DST_PTZ"

udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=tty
sleep 2

echo
echo "==== 结果 ===="
ok=1
for link in /dev/ptz485 /dev/imu_usb /dev/gq_otk9633; do
    if [[ -L "$link" ]]; then
        echo "  $link -> $(readlink -f "$link")"
    else
        echo "  $link 不存在（把该适配器拔插一次）"
        [[ "$link" == /dev/gq_otk9633 ]] || ok=0
    fi
done

# 交叉检查：两个别名不能指到同一个 tty
if [[ -L /dev/ptz485 && -L /dev/imu_usb ]]; then
    p=$(readlink -f /dev/ptz485); i=$(readlink -f /dev/imu_usb)
    if [[ "$p" == "$i" ]]; then
        echo "  ⚠️  /dev/ptz485 和 /dev/imu_usb 指向同一个口（$p），检查 USB 是否插反" >&2
        ok=0
    fi
fi

if [[ $ok -eq 1 && -L /dev/ptz485 ]]; then
    echo
    echo "验证云台:"
    echo "  python3 $HERE/ptz_pelco_probe.py --port /dev/ptz485 --baud 2400 --addr 1 --read"
fi
exit $((1 - ok))
