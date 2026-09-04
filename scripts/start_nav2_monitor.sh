#!/bin/bash
# Nav2 全栈 + 远程监控台 一键启动（Ctrl-C 全部停止）
# 含：仿真 + Nav2(AMCL/规划/控制) + 遥控遥测网关 + 网页 + MediaMTX + D435推流
# 浏览器访问: http://localhost:8888/monitor.html
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

export ROBOT_CONFIG="$ROOT/config/config.yaml"

# 仿真 + Nav2 全栈
ros2 launch chassis_nav2 nav2.launch.py &
sleep 1

# 遥控/遥测网关（初始定位、目标点下发）
ros2 run robot_gateway gateway --ros-args -p use_sim_time:=true &

# 网页服务
python3 -m http.server 8888 --directory "$ROOT/web" &

# 视频链：MediaMTX + D435 推流（相机未 attach 时推流脚本会退出，不影响其他功能）
"$ROOT/bin/mediamtx" "$ROOT/deploy/mediamtx.yml" &
sleep 1
bash "$ROOT/scripts/push_d435_wsl.sh" &

# 图像识别节点（默认从 MediaMTX 拉流；离线测试用 -p video_source:=<视频文件> 覆盖）
ros2 run detector_node detector &

# 抓拍服务节点（手动抓拍/识别联动/巡逻到点抓拍共用 /capture_request 接口）
ros2 run capture_server capture_server &

trap 'kill $(jobs -p) 2>/dev/null || true' EXIT
echo "=========================================================="
echo " Nav2 全栈监控台已启动: http://localhost:8888/monitor.html"
echo " 注意：重启本脚本后若页面无数据，等 ~40s Nav2 拉起即可"
echo "=========================================================="
wait
