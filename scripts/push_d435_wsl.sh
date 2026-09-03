#!/bin/bash
# WSL2 + D435(usbipd 透传) 推流：640x480 yuyv 推到本机 MediaMTX
# 说明：usbipd 带宽有限，720p yuyv(~55MB/s) 会频繁出现帧损坏，640x480(~18MB/s) 实测稳定；
#       D435 经 usbipd 不暴露 MJPEG，只有 yuyv422，彩色流节点为 /dev/video4（可用 DEV 覆盖）。
set -e
DEV="${DEV:-/dev/video4}"
SIZE="${SIZE:-640x480}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-1200k}"

echo "push_d435_wsl: $DEV $SIZE@$FPS -> rtsp://127.0.0.1:8554/cam"
exec ffmpeg -hide_banner -loglevel warning \
  -f v4l2 -input_format yuyv422 -video_size "$SIZE" -framerate "$FPS" -i "$DEV" \
  -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p -g 60 -b:v "$BITRATE" \
  -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/cam
