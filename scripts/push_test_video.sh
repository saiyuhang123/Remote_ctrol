#!/bin/bash
# 测试视频源（无摄像头阶段）：720p 测试图案 -> x264 软编 -> RTSP 推到本机 MediaMTX
# 摄像头到位后改为此设备采集；编码器按 config/config.yaml 的 video.encoder 切换
# （Jetson 硬编 nvv4l2h264enc / x64 核显 h264_vaapi / 兜底 libx264）
set -e
exec ffmpeg -hide_banner -loglevel warning -re \
  -f lavfi -i testsrc2=size=1280x720:rate=25 \
  -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p -g 50 -b:v 1500k \
  -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/cam
