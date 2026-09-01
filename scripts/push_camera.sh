#!/bin/bash
# 摄像头推流：自动探测 UVC 彩色摄像头（优先 MJPG，其次 YUYV），720p 推到本机 MediaMTX
# 找不到摄像头时退出码 1，由 start_m1.sh 回退到测试源
# 编码器后续按 config/config.yaml 的 video.encoder 切换硬编，M1 阶段统一 libx264 软编
set -e
DEV="" FMT=""
for v in /dev/video*; do
  [ -e "$v" ] || continue
  if timeout 5 ffmpeg -hide_banner -f v4l2 -list_formats all -i "$v" 2>&1 | grep -qi 'mjpeg'; then
    DEV="$v"; FMT=mjpeg; break
  fi
done
if [ -z "$DEV" ]; then
  for v in /dev/video*; do
    [ -e "$v" ] || continue
    if timeout 5 ffmpeg -hide_banner -f v4l2 -list_formats all -i "$v" 2>&1 | grep -qi 'yuyv422'; then
      DEV="$v"; FMT=yuyv422; break
    fi
  done
fi
[ -z "$DEV" ] && { echo "push_camera: 未发现可用摄像头"; exit 1; }

echo "push_camera: 使用 $DEV ($FMT 1280x720@30)"
exec ffmpeg -hide_banner -loglevel warning \
  -f v4l2 -input_format "$FMT" -video_size 1280x720 -framerate 30 -i "$DEV" \
  -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p -g 60 -b:v 1800k \
  -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/cam
