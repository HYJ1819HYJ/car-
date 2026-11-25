#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
docker_dir="/home/asuka/Documents/Codex/2026-10-02/24-04-ros1-ros2-cubemx-clino/outputs/ros-docker"
rosctl="$docker_dir/rosctl"
container="asuka-ros1-noetic"
base_device="${ROBOT_BASE_DEVICE:-/dev/serial/by-path/pci-0000:00:14.0-usb-0:6:1.0-port0}"
gimbal_device="${ROBOT_GIMBAL_DEVICE:-/dev/serial/by-path/pci-0000:00:14.0-usb-0:7:1.0-port0}"
camera_device="${ROBOT_CAMERA_DEVICE:-/dev/v4l/by-id/usb-HRY_USB_Camera_20191109-video-index0}"
log_dir="$project_dir/outputs/direct-visual"
mode="${1:-start}"

usage() {
  cat <<'EOF'
用法：
  ./start-blue-direct.sh           启动或重新开始一次蓝纸追踪
  ./start-blue-direct.sh --stop    停止底盘自主追踪
  ./start-blue-direct.sh --check   只查看设备和当前任务状态，不启动车辆

启动后云台搜索蓝纸，小车根据画面与雷达直接控制，无需地图或 RViz。
蓝纸面积至少 16% 且车头误差不超过 5° 时停车；前向急停 12 cm。
换角度后，将车放稳，再次运行脚本即可开始下一次。
窗口关闭后不影响追踪；--stop 停止底盘，摄像头和云台仍可继续运行。
EOF
}

case "$mode" in
  -h|--help) usage; exit 0 ;;
  start|--stop|--check) ;;
  *) usage >&2; exit 2 ;;
esac
if [[ $# -gt 1 ]]; then usage >&2; exit 2; fi
command -v docker >/dev/null || { printf '未找到 Docker。\n' >&2; exit 1; }
[[ -x "$rosctl" ]] || { printf '找不到 ROS Docker 工具：%s\n' "$rosctl" >&2; exit 1; }

ros1() {
  "$rosctl" exec1 bash -c '
    set -e
    source /opt/ros/noetic/setup.bash
    test "$ROS_DISTRO" = noetic
    source /home/rosdev/LakiBeam_ws/devel/setup.bash
    if [[ -f /workspace/xiaoche/devel/setup.bash ]]; then
      source /workspace/xiaoche/devel/setup.bash
    fi
    export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
    exec "$@"
  ' _ "$@"
}

container_running() {
  [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" == true ]]
}

devices_present() {
  [[ -c "$base_device" && -c "$gimbal_device" && -c "$camera_device" ]]
}

device_mapping_matches() {
  container_running && devices_present || return 1
  local host_base host_gimbal host_camera mapped
  host_base="$(stat -Lc '%t:%T' "$base_device")"
  host_gimbal="$(stat -Lc '%t:%T' "$gimbal_device")"
  host_camera="$(stat -Lc '%t:%T' "$camera_device")"
  mapped="$(docker exec "$container" bash -c 'stat -Lc "%t:%T" /dev/ttyUSB0 /dev/ttyUSB1 /dev/video4' 2>/dev/null || true)"
  [[ "$mapped" == "$(printf '%s\n%s\n%s' "$host_base" "$host_gimbal" "$host_camera")" ]]
}

node_list() {
  ros1 timeout 3 rosnode list 2>/dev/null || true
}

publish_enable() {
  ros1 python3 - "$1" <<'PY'
import sys, time
import rospy
from std_msgs.msg import Bool
value = sys.argv[1] == 'true'
rospy.init_node('direct_visual_start_request', anonymous=True)
publisher = rospy.Publisher('/blue_autonomy/enable', Bool, queue_size=1)
deadline = time.monotonic() + 5
while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
    time.sleep(.05)
if publisher.get_num_connections() == 0:
    raise SystemExit('直接追踪节点未连接，未发送启动指令。')
publisher.publish(Bool(data=value))
time.sleep(.4)
PY
}

if [[ "$mode" == --check ]]; then
  if devices_present; then
    printf '底盘串口、云台串口和摄像头：设备存在。\n'
  else
    printf '设备未全部就绪：\n  底盘 %s\n  云台 %s\n  摄像头 %s\n' "$base_device" "$gimbal_device" "$camera_device"
  fi
  if device_mapping_matches; then
    printf 'ROS1 容器设备映射：匹配。\n'
  else
    printf 'ROS1 容器设备映射：未就绪或不匹配。\n'
  fi
  if container_running; then
    nodes="$(node_list)"
    if [[ "$nodes" == *$'\n/direct_visual_controller'* || "$nodes" == /direct_visual_controller* ]]; then
      ros1 timeout 4 rostopic echo -n 1 /blue_autonomy/state || true
    else
      printf '直接追踪节点当前未运行。\n'
    fi
  fi
  exit 0
fi

if [[ "$mode" == --stop ]]; then
  if ! container_running || [[ "$(node_list)" != *'/direct_visual_controller'* ]]; then
    printf '直接追踪节点没有运行。\n'
    exit 0
  fi
  publish_enable false
  printf '已停止底盘自主追踪；摄像头和云台仍在运行。\n'
  exit 0
fi

if ! devices_present; then
  printf '未找到全部设备，请检查连接：\n  底盘 %s\n  云台 %s\n  摄像头 %s\n' "$base_device" "$gimbal_device" "$camera_device" >&2
  exit 1
fi
if [[ "$(stat -Lc '%t:%T' "$base_device")" == "$(stat -Lc '%t:%T' "$gimbal_device")" ]]; then
  printf '底盘与云台串口指向了同一个设备，请检查 USB 连接。\n' >&2
  exit 1
fi

if ! container_running; then
  printf '启动 ROS1 容器……\n'
  if ! "$rosctl" up; then
    printf '完整 ROS 环境启动未完成，继续单独准备 ROS1。\n'
  fi
fi

if ! device_mapping_matches; then
  if container_running && [[ -n "$(node_list)" ]]; then
    printf 'ROS1 当前已有节点运行，且设备映射已变化；请先关闭当前 ROS1 任务再重试。\n' >&2
    exit 1
  fi
  printf '更新 ROS1 容器的底盘、云台和摄像头映射……\n'
  compose=(docker compose --project-directory "$docker_dir" --env-file "$docker_dir/.env" -f "$docker_dir/compose.yaml")
  if [[ -f "$docker_dir/data/gui/enabled" ]]; then
    compose+=(--env-file "$docker_dir/data/gui/gui.env" -f "$docker_dir/compose.gui.yaml")
  fi
  if [[ -f "$docker_dir/data/peripherals/enabled" ]]; then
    compose+=(-f "$docker_dir/compose.peripherals.yaml")
  fi
  compose+=(-f "$project_dir/tools/direct_visual_devices.compose.yaml")
  ROBOT_BASE_DEVICE="$base_device" ROBOT_GIMBAL_DEVICE="$gimbal_device" ROBOT_CAMERA_DEVICE="$camera_device" \
    "${compose[@]}" up -d --no-deps --force-recreate ros1
  if ! device_mapping_matches; then
    printf 'ROS1 容器设备映射仍未就绪，请检查连接。\n' >&2
    exit 1
  fi
fi

if [[ "$(ip -o -4 addr show 2>/dev/null)" != *'192.168.198.1/24'* ]]; then
  printf '连接雷达有线网络……\n'
  nmcli connection up LakiBeam-RJ45 ifname enp0s31f6 >/dev/null || {
    printf '雷达网络未连接，请检查网线和 LakiBeam-RJ45 网络配置。\n' >&2
    exit 1
  }
fi

if ! ros1 test -x /workspace/xiaoche/devel/lib/maze_planner/direct_visual_controller; then
  printf '编译直接追踪程序……\n'
  ros1 bash -c 'cd /workspace/xiaoche && catkin_make --pkg stm32_bridge maze_planner -j4'
fi

mkdir -p "$log_dir"
run_tag="$(date +%Y%m%d-%H%M%S)"
launch_log="$log_dir/launch-$run_tag.log"
view_log="$log_dir/view-$run_tag.log"
nodes="$(node_list)"
for name in /move_base /blue_goal_navigator; do
  if [[ $'\n'"$nodes"$'\n' == *$'\n'"$name"$'\n'* ]]; then
    printf '检测到旧导航节点 %s，请先关闭原来的小车任务再启动。\n' "$name" >&2
    exit 1
  fi
done
if [[ "$nodes" != *'/direct_visual_controller'* ]]; then
  for name in /serial_bridge /richbeam_lidar /gimbal_bridge /usb_cam; do
    if [[ $'\n'"$nodes"$'\n' == *$'\n'"$name"$'\n'* ]]; then
      printf '检测到已有 ROS 节点 %s，请先关闭原来的小车任务再启动。\n' "$name" >&2
      exit 1
    fi
  done
  printf '启动摄像头、云台、雷达和直接控制程序……\n'
  nohup "$rosctl" exec1 bash -c '
    set -e
    source /opt/ros/noetic/setup.bash
    test "$ROS_DISTRO" = noetic
    source /home/rosdev/LakiBeam_ws/devel/setup.bash
    source /workspace/xiaoche/devel/setup.bash
    export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
    exec roslaunch -p 11311 maze_bridge direct_visual_tracking.launch auto_start:=false
  ' >"$launch_log" 2>&1 </dev/null &
  launch_wrapper=$!
  ready=false
  for _ in {1..50}; do
    nodes="$(node_list)"
    if [[ "$nodes" == *'/direct_visual_controller'* ]]; then ready=true; break; fi
    if ! kill -0 "$launch_wrapper" 2>/dev/null; then break; fi
    sleep .2
  done
  if [[ "$ready" != true ]]; then
    printf '直接追踪程序未能启动，请查看 %s\n' "$launch_log" >&2
    exit 1
  fi
fi

nodes="$(node_list)"
viewer_available=true
if [[ "$nodes" != *'/direct_tracking_view'* && "$nodes" != *'/finish_tracking_view_'* ]]; then
  nohup "$rosctl" exec1 bash -c '
    set -e
    source /opt/ros/noetic/setup.bash
    test "$ROS_DISTRO" = noetic
    source /home/rosdev/LakiBeam_ws/devel/setup.bash
    source /workspace/xiaoche/devel/setup.bash
    export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
    exec python3 /workspace/xiaoche/tools/direct_tracking_view.py
  ' >"$view_log" 2>&1 </dev/null &
  viewer_wrapper=$!
  sleep .5
  if ! kill -0 "$viewer_wrapper" 2>/dev/null; then
    viewer_available=false
    printf '摄像头窗口未能打开，详情见 %s；追踪程序仍可运行。\n' "$view_log" >&2
  fi
fi

state="$(ros1 timeout 4 rostopic echo -n 1 /blue_autonomy/state 2>/dev/null || true)"
if [[ "$state" != *'IDLE:'* && "$state" != *'COMPLETE:'* && -n "$state" ]]; then
  printf '直接追踪已经在运行：%s\n' "$state"
  exit 0
fi
publish_enable true
printf '已启动蓝纸追踪。\n'
if [[ "$viewer_available" == true ]]; then
  printf '摄像头窗口显示目标框和占比。\n'
fi
printf '停止：%s --stop\n' "$project_dir/start-blue-direct.sh"
