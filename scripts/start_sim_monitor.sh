#!/bin/bash
# 仿真 + 远程监控台 一键启动（Ctrl-C 全部停止）
# 浏览器访问: http://localhost:8888/monitor.html
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

export ROBOT_CONFIG="$ROOT/config/config.yaml"

ros2 launch chassis_gazebo sim.launch.py &
sleep 1
ros2 run robot_gateway gateway --ros-args -p use_sim_time:=true &
python3 -m http.server 8888 --directory "$ROOT/web" &

# 视频链：MediaMTX + D435 推流（相机未接入时推流脚本会报错退出，不影响其余功能）
"$ROOT/bin/mediamtx" "$ROOT/deploy/mediamtx.yml" &
sleep 1
bash "$ROOT/scripts/push_d435_wsl.sh" &

trap 'kill $(jobs -p) 2>/dev/null || true' EXIT
echo "=========================================================="
echo " 仿真监控台已启动。浏览器访问: http://localhost:8888/monitor.html"
echo " （真车模式：去掉网关的 use_sim_time 参数即可）"
echo "=========================================================="
wait
