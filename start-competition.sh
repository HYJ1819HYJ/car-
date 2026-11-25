#!/usr/bin/env bash
# Start the mapped search and automatic visual parking without an RViz goal.
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
docker_dir="/home/asuka/Documents/Codex/2026-10-02/24-04-ros1-ros2-cubemx-clino/outputs/ros-docker"
rosctl="$docker_dir/rosctl"
container="asuka-ros1-noetic"
runtime_dir="$project_dir/outputs/competition-run"
map_file="${COMPETITION_MAP_FILE:-$project_dir/maps/20261125/new_field/map.yaml}"
map_file="$(realpath -m -- "$map_file")"
map_dir="$(dirname -- "$map_file")"
pose_file="${COMPETITION_START_POSE:-$map_dir/start_pose.json}"
course_file="${COMPETITION_COURSE_FILE:-$map_dir/course_waypoints.yaml}"
camera_device="${ROBOT_CAMERA_DEVICE:-/dev/v4l/by-id/usb-HRY_USB_Camera_20191109-video-index0}"
mode="${1:-start}"

usage() {
  cat <<'EOF'
用法：
  ./start-competition.sh          从已记录的起点启动自主搜索和停车
  ./start-competition.sh --check  查看地图、起点、路线和设备，不启动车辆
  ./start-competition.sh --stop   停止本脚本启动的任务

默认使用 maps/20261125/new_field 的地图、起点和建图路线，无需在 RViz 指定终点。
启动后沿建图路线导航；识别蓝纸且雷达通道满足条件后切换到视觉追踪。
前向急停 12 cm；蓝纸轮廓占比达到 16% 且朝向误差不超过 5° 时停车。
在运行终端按 Ctrl+C，或另开终端执行 --stop，可停止任务。
换地图时，可设置 COMPETITION_MAP_FILE、COMPETITION_START_POSE 和 COMPETITION_COURSE_FILE。
EOF
}

case "$mode" in start|--check|--stop) ;; -h|--help) usage; exit 0 ;; *) usage >&2; exit 2 ;; esac
[[ $# -le 1 ]] || { usage >&2; exit 2; }
mkdir -p "$runtime_dir"

container_running() {
  [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" == true ]]
}

ros1() {
  "$rosctl" exec1 bash -c '
    set -e
    source /opt/ros/noetic/setup.bash
    test "$ROS_DISTRO" = noetic
    source /home/rosdev/LakiBeam_ws/devel/setup.bash
    source /workspace/xiaoche/devel/setup.bash
    export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
    exec "$@"
  ' _ "$@"
}

stop_pidfile() {
  local file="$1" expected="$2" pid args
  [[ -s "$file" ]] || return 1
  read -r pid < "$file" || return 1
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  args="$(docker exec "$container" ps -p "$pid" -o args= 2>/dev/null || true)"
  if [[ "$args" == *"$expected"* ]]; then
    docker exec "$container" kill -INT "$pid" >/dev/null 2>&1
    return
  fi
  return 1
}

if [[ "$mode" == --stop ]]; then
  if ! container_running; then
    printf 'ROS1 容器没有运行。\n'
    exit 0
  fi
  requested=false
  nodes="$(ros1 timeout 3 rosnode list 2>/dev/null || true)"
  if grep -qx '/visual_handoff_manager' <<< "$nodes"; then
    # The existing manager cancels navigation and publishes zero when disabled.
    if ros1 timeout 4 rostopic pub -1 /blue_autonomy/enable std_msgs/Bool 'data: false' >/dev/null 2>&1; then requested=true; fi
  fi
  if stop_pidfile "$runtime_dir/launch.pid" 'navigation_visual_handoff.launch'; then requested=true; fi
  stop_pidfile "$runtime_dir/viewer.pid" 'direct_tracking_view.py' || true
  if [[ "$requested" == true ]]; then
    printf '已发送停止请求，请确认车辆已停稳。\n'
  else
    printf '没有发现可停止的自主任务。\n'
  fi
  exit 0
fi

detect_serial() {
  local expected="$1" device info revision vendor model
  local -a matches=()
  for device in /dev/ttyUSB*; do
    [[ -c "$device" ]] || continue
    info="$(udevadm info -q property -n "$device" 2>/dev/null || true)"
    revision="$(awk -F= '$1=="ID_REVISION" {print $2}' <<< "$info")"
    vendor="$(awk -F= '$1=="ID_VENDOR_ID" {print $2}' <<< "$info")"
    model="$(awk -F= '$1=="ID_MODEL_ID" {print $2}' <<< "$info")"
    [[ "$vendor" == 1a86 && "$model" == 7523 && "$revision" == "$expected" ]] && matches+=("$device")
  done
  if [[ ${#matches[@]} != 1 ]]; then
    printf '无法唯一识别 USB 转串口（revision %s）；请核对设备，或设置 ROBOT_BASE_DEVICE 和 ROBOT_GIMBAL_DEVICE。\n' "$expected" >&2
    return 1
  fi
  printf '%s\n' "${matches[0]}"
}

base_device="${ROBOT_BASE_DEVICE:-$(detect_serial 8132)}"
gimbal_device="${ROBOT_GIMBAL_DEVICE:-$(detect_serial 8134)}"
for device in "$base_device" "$gimbal_device" "$camera_device"; do
  [[ -c "$device" ]] || { printf '设备不存在：%s\n' "$device" >&2; exit 1; }
done
[[ "$(stat -Lc '%t:%T' "$base_device")" != "$(stat -Lc '%t:%T' "$gimbal_device")" ]] || {
  printf '底盘与云台指向同一串口，请检查连接。\n' >&2; exit 1;
}
[[ -f "$map_file" && -f "$pose_file" && -f "$course_file" ]] || { printf '地图、起点或路线文件不存在：%s / %s / %s\n' "$map_file" "$pose_file" "$course_file" >&2; exit 1; }
map_file="$(realpath "$map_file")"
pose_file="$(realpath "$pose_file")"
course_file="$(realpath "$course_file")"
[[ "$map_file" == "$project_dir/"* ]] || { printf '地图需位于工作区内，供 ROS1 容器读取。\n' >&2; exit 1; }
[[ "$course_file" == "$project_dir/"* ]] || { printf '路线需位于工作区内，供 ROS1 容器读取。\n' >&2; exit 1; }
container_map="/workspace/xiaoche/${map_file#"$project_dir/"}"
container_course="/workspace/xiaoche/${course_file#"$project_dir/"}"
pose_values="$(python3 - "$pose_file" <<'PY'
import json, math, sys
with open(sys.argv[1]) as stream:
    data = json.load(stream)
values = [float(data[key]) for key in ('x', 'y', 'yaw')]
if not all(math.isfinite(value) for value in values):
    raise SystemExit('起点位姿包含无效数值')
print(*values)
PY
)"
read -r start_x start_y start_yaw <<< "$pose_values"
course_count="$(python3 - "$course_file" <<'PY'
import math, sys, yaml
with open(sys.argv[1]) as stream:
    data = yaml.safe_load(stream)
points = data.get('course_waypoints') if isinstance(data, dict) else None
if not isinstance(points, list) or not points:
    raise SystemExit('路线必须包含至少一个 course_waypoints 途经点')
for point in points:
    if (not isinstance(point, list) or len(point) != 2 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) for v in point)):
        raise SystemExit('路线途经点必须是有限数值组成的 [x, y]')
print(len(points))
PY
)"

device_mapping_matches() {
  container_running || return 1
  local mapped expected
  expected="$(stat -Lc '%t:%T' "$base_device" "$gimbal_device" "$camera_device")"
  mapped="$(docker exec "$container" bash -c 'stat -Lc "%t:%T" /dev/ttyUSB0 /dev/ttyUSB1 /dev/video4' 2>/dev/null || true)"
  [[ "$mapped" == "$expected" ]]
}

if [[ "$mode" == --check ]]; then
  printf '地图：%s\n起点：%s（%s）\n路线：%s（%s 个途经点）\n底盘：%s；云台：%s；摄像头：%s\n' "$map_file" "$pose_file" "$pose_values" "$course_file" "$course_count" "$base_device" "$gimbal_device" "$camera_device"
  if device_mapping_matches; then printf 'ROS1 容器设备映射：匹配。\n'; else printf 'ROS1 容器设备映射将在启动时更新。\n'; fi
  if [[ -r /sys/class/net/enp0s31f6/carrier ]]; then printf '雷达网口链路：%s\n' "$(cat /sys/class/net/enp0s31f6/carrier)"; fi
  exit 0
fi

exec 9>"$runtime_dir/start.lock"
flock -n 9 || { printf '已有一键比赛任务正在运行。\n' >&2; exit 1; }

if ! container_running; then
  printf '启动 ROS1 容器……\n'
  # rosctl also starts ROS2; its unrelated missing devices do not prevent a
  # subsequent ROS1-only device refresh.
  "$rosctl" up >"$runtime_dir/container-start.log" 2>&1 || true
fi
if container_running && [[ -n "$(ros1 timeout 3 rosnode list 2>/dev/null || true)" ]]; then
  printf 'ROS1 已有节点运行，请先停止当前小车任务再启动。\n' >&2
  exit 1
fi
if ! device_mapping_matches; then
  printf '按当前连接更新 ROS1 设备映射……\n'
  compose=(docker compose --project-directory "$docker_dir" --env-file "$docker_dir/.env" -f "$docker_dir/compose.yaml")
  if [[ -f "$docker_dir/data/gui/enabled" ]]; then
    compose+=(--env-file "$docker_dir/data/gui/gui.env" -f "$docker_dir/compose.gui.yaml")
  fi
  if [[ -f "$docker_dir/data/peripherals/enabled" ]]; then compose+=(-f "$docker_dir/compose.peripherals.yaml"); fi
  compose+=(-f "$project_dir/tools/direct_visual_devices.compose.yaml")
  ROBOT_BASE_DEVICE="$base_device" ROBOT_GIMBAL_DEVICE="$gimbal_device" ROBOT_CAMERA_DEVICE="$camera_device" \
    "${compose[@]}" up -d --no-deps --force-recreate ros1
  device_mapping_matches || { printf 'ROS1 设备映射未就绪。\n' >&2; exit 1; }
fi

if [[ "$(ip -o -4 addr show enp0s31f6 2>/dev/null)" != *'192.168.198.1/24'* ]]; then
  nmcli connection up LakiBeam-RJ45 ifname enp0s31f6 >/dev/null
fi
[[ "$(cat /sys/class/net/enp0s31f6/carrier)" == 1 ]] || { printf '雷达网口没有物理链路。\n' >&2; exit 1; }
python3 "$project_dir/../tools/refresh_ros_gui_auth.py" "$docker_dir/data/gui" || true

rm -f "$runtime_dir/launch.pid" "$runtime_dir/viewer.pid"
run_tag="$(date +%Y%m%d-%H%M%S)"
launch_log="$runtime_dir/launch-$run_tag.log"
viewer_log="$runtime_dir/viewer-$run_tag.log"
printf '启动自主任务：小车沿建图路线导航，云台搜索，终点由蓝纸识别。\n'
printf '地图：%s\n日志：%s\n停止：Ctrl+C 或本脚本 --stop\n' "$map_file" "$launch_log"

ros1 bash -c '
  printf "%s\n" "$$" > /workspace/xiaoche/outputs/competition-run/launch.pid
  exec roslaunch -p 11311 maze_bridge navigation_visual_handoff.launch \
    map_file:="$1" initial_pose_x:="$2" initial_pose_y:="$3" initial_pose_a:="$4" \
    course_waypoints_file:="$5" \
    auto_start:=true show_rviz:=false base_port:=/dev/ttyUSB0 \
    gimbal_port:=/dev/ttyUSB1 camera_device:=/dev/video4
' _ "$container_map" "$start_x" "$start_y" "$start_yaw" "$container_course" >"$launch_log" 2>&1 9>&- &
launch_wrapper=$!

cleanup() {
  trap - EXIT INT TERM
  if container_running; then
    ros1 timeout 4 rostopic pub -1 /blue_autonomy/enable std_msgs/Bool 'data: false' >/dev/null 2>&1 || true
    stop_pidfile "$runtime_dir/launch.pid" 'navigation_visual_handoff.launch' || true
    stop_pidfile "$runtime_dir/viewer.pid" 'direct_tracking_view.py' || true
  fi
  wait "$launch_wrapper" 2>/dev/null || true
  rm -f "$runtime_dir/launch.pid" "$runtime_dir/viewer.pid"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "${COMPETITION_SHOW_CAMERA:-1}" == 1 ]]; then
  for _ in {1..50}; do
    kill -0 "$launch_wrapper" 2>/dev/null || break
    if ros1 timeout 1 rostopic type /usb_cam/image_raw >/dev/null 2>&1; then break; fi
    sleep .1
  done
  if kill -0 "$launch_wrapper" 2>/dev/null; then
    ros1 bash -c '
      printf "%s\n" "$$" > /workspace/xiaoche/outputs/competition-run/viewer.pid
      exec python3 /workspace/xiaoche/tools/direct_tracking_view.py
    ' >"$viewer_log" 2>&1 9>&- &
  fi
fi

wait "$launch_wrapper"
