#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image=xiaoche/ros1-noetic:latest
lidar_ws="${LAKIBEAM_WS:-$HOME/LakiBeam_ws}"
build_dir="$root/outputs/competition-noetic"
container=car-competition
rviz_container=car-competition-rviz
bag_container=car-competition-bag
action="${1:-help}"
field="${2:-}"

if [[ "$action" == help || "$action" == --help ]]; then
  printf '用法：bash tools/competition_terminal.sh prepare\n'
  printf '      bash tools/competition_terminal.sh {map|rviz|save|ready|status|go|record|stop} 场次名\n'
  exit 0
fi

mounts=(
  --mount "type=bind,src=$root,dst=/workspace/xiaoche,readonly"
  --mount "type=bind,src=$lidar_ws,dst=/workspace/LakiBeam_ws,readonly"
)

if [[ "$action" == prepare ]]; then
  mkdir -p "$build_dir"
  docker run --rm --network none --user "$(id -u):$(id -g)" \
    -e ROS_HOME=/workspace/compiled/.ros "${mounts[@]}" \
    --mount "type=bind,src=$build_dir,dst=/workspace/compiled" "$image" bash -c '
      set -e
      source /opt/ros/noetic/setup.bash
      test "$ROS_DISTRO" = noetic
      source /workspace/LakiBeam_ws/devel/setup.bash
      rospack find lakibeam1
      cd /workspace/compiled
      catkin_make --source /workspace/xiaoche/src --build build -DCMAKE_BUILD_TYPE=Release -j4
    '
  exit 0
fi

case "$action" in
  map|rviz|turn|save|ready|status|go|record|stop) ;;
  *) printf '未知命令：%s\n' "$action" >&2; exit 2 ;;
esac
[[ $# -eq 2 && "$field" =~ ^[[:alnum:]_][[:alnum:]_.-]*$ ]] || {
  printf '请提供场次名，例如 retest-01。\n' >&2
  exit 2
}
field_dir="$root/outputs/competition-fields/$field"

serial_device() {
  local expected="$1" node
  local -a matches=()
  for node in /dev/ttyUSB*; do
    [[ -c "$node" ]] || continue
    if udevadm info -q property -n "$node" | grep -Fxq "ID_REVISION=$expected"; then
      matches+=("$node")
    fi
  done
  if [[ ${#matches[@]} -ne 1 ]]; then
    printf '无法唯一识别串口 revision %s，请核对设备。\n' "$expected" >&2
    return 1
  fi
  printf '%s\n' "${matches[0]}"
}

ros() {
  docker exec "$container" bash -c '
    source /opt/ros/noetic/setup.bash && test "$ROS_DISTRO" = noetic &&
    source /workspace/LakiBeam_ws/devel/setup.bash &&
    source /workspace/compiled/devel/setup.bash &&
    exec "$@"
  ' _ "$@"
}

require_phase() {
  local phase="$1" live
  live="$(docker inspect --format '{{ index .Config.Labels "competition.field" }} {{ index .Config.Labels "competition.phase" }}' "$container" 2>/dev/null || true)"
  if [[ "$live" != "$field $phase" ]]; then
    printf '场次 %s 的 %s 进程未启动。\n' "$field" "$phase" >&2
    exit 1
  fi
}

rviz_display_options() {
  local authority="${XAUTHORITY:-$HOME/.Xauthority}"
  if [[ -z "${DISPLAY:-}" || ! -d /tmp/.X11-unix || ! -r "$authority" ]]; then
    printf '无法连接桌面显示，请从本机图形终端运行。\n' >&2
    return 1
  fi
  gui=(--uts host -e "DISPLAY=$DISPLAY" -e XAUTHORITY=/tmp/competition.Xauthority
       -e QT_X11_NO_MITSHM=1
       --tmpfs "/.rviz:uid=$(id -u),gid=$(id -g),mode=0700"
       --mount "type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly"
       --mount "type=bind,src=$(realpath "$authority"),dst=/tmp/competition.Xauthority,readonly")
}

close_mapping_viewer() {
  docker stop --signal INT --timeout 5 "$rviz_container" >/dev/null 2>&1 || true
}

check_navigation_scan() {
  if ! ros timeout 4 rostopic echo -n 1 /scan/header >/dev/null; then
    printf '雷达没有实时输出，请检查雷达电源和网线。\n' >&2
    return 1
  fi
}

start_navigation_recording() {
  local running bag_field bag_name bag_dir requested="${1:-}"
  running="$(docker inspect --format '{{ index .Config.Labels "competition.field" }} {{ index .Config.Labels "competition.bag" }}' "$bag_container" 2>/dev/null || true)"
  if [[ -n "$running" ]]; then
    read -r bag_field bag_name <<< "$running"
    if [[ "$bag_field" != "$field" || -n "$requested" ]]; then
      printf '场次 %s 正在录包，请先结束该场次。\n' "$bag_field" >&2
      return 1
    fi
    bag_dir="$field_dir/bags/$bag_name"
  else
    mkdir -p "$field_dir/bags"
    bag_dir="${requested:-$(mktemp -d "$field_dir/bags/navigation-$(date +%Y%m%d-%H%M%S)-XXXXXX")}"
    docker run --detach --rm --init --name "$bag_container" --network host \
      --label "competition.field=$field" --label "competition.bag=${bag_dir##*/}" \
      --user "$(id -u):$(id -g)" \
      --mount "type=bind,src=$bag_dir,dst=/workspace/recording" \
      --mount "type=bind,src=$root/tools/record_navigation_session.py,dst=/workspace/record_navigation_session.py,readonly" \
      -e ROS_HOME=/workspace/recording/.ros -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
      "$image" bash -c '
        source /opt/ros/noetic/setup.bash && test "$ROS_DISTRO" = noetic &&
        exec python3 /workspace/record_navigation_session.py --worker /workspace/recording --minutes 0
      ' >/dev/null
  fi
  if ! timeout 12 bash -c 'until compgen -G "$1/*.bag.active" >/dev/null; do sleep 0.2; done' _ "$bag_dir"; then
    printf '录包没有开始，请查看 %s 中的日志。\n' "$bag_dir" >&2
    docker stop --signal INT --timeout 35 "$bag_container" >/dev/null 2>&1 || true
    return 1
  fi
  printf 'rosbag 录制目录：%s\n' "$bag_dir"
}

case "$action" in
  map|ready)
    [[ -f "$build_dir/devel/setup.bash" ]] || {
      printf '请先运行 prepare。\n' >&2; exit 1
    }
    if docker inspect "$container" >/dev/null 2>&1 || [[ -n "$(ss -Hlt 'sport = :11311')" ]]; then
      printf '已有 ROS 任务运行，请先停止。\n' >&2; exit 1
    fi
    base="${ROBOT_BASE_DEVICE:-$(serial_device 8132)}"
    [[ -c "$base" ]] || { printf '底盘设备不存在：%s\n' "$base" >&2; exit 1; }
    devices=(--device "$base:/dev/ttyUSB0" --group-add "$(stat -Lc %g "$base")")
    gui=()
    if [[ "$action" == map ]]; then
      if [[ -c /dev/input/js0 ]]; then
        devices+=(--device /dev/input/js0:/dev/input/js0:r --group-add "$(stat -Lc %g /dev/input/js0)")
      fi
      show_rviz="${COMPETITION_SHOW_RVIZ:-true}"
      if [[ "$show_rviz" == true ]]; then rviz_display_options; fi
      mkdir -p "$(dirname "$field_dir")"
      mkdir "$field_dir" || { printf '场次已存在，请使用新的场次名。\n' >&2; exit 1; }
      phase=mapping
      launch=(roslaunch -p 11311 maze_bridge slam_toolbox_build_map.launch
              port:=/dev/ttyUSB0 show_rviz:="$show_rviz" use_camera:=false)
    else
      for name in map.yaml map.pgm start_pose.json; do
        [[ -s "$field_dir/$name" ]] || { printf '缺少 %s，请完成 save。\n' "$name" >&2; exit 1; }
      done
      gimbal="${ROBOT_GIMBAL_DEVICE:-$(serial_device 8134)}"
      camera="${ROBOT_CAMERA_DEVICE:-/dev/v4l/by-id/usb-HRY_USB_Camera_20191109-video-index0}"
      [[ -c "$gimbal" && -c "$camera" && "$(stat -Lc %t:%T "$base")" != "$(stat -Lc %t:%T "$gimbal")" ]] || {
        printf '请检查底盘、云台和相机的设备映射。\n' >&2; exit 1
      }
      devices+=(--device "$gimbal:/dev/ttyUSB1" --group-add "$(stat -Lc %g "$gimbal")"
                --device "$(readlink -f "$camera"):/dev/video4" --group-add "$(stat -Lc %g "$camera")")
      pose="$(python3 -c 'import json,sys; p=json.load(open(sys.argv[1])); print(p["x"],p["y"],p["yaw"])' "$field_dir/start_pose.json")"
      read -r x y yaw <<< "$pose"
      phase=navigation
      launch=(roslaunch -p 11311 maze_bridge navigation_visual_handoff.launch
              map_file:=/workspace/field/map.yaml
              initial_pose_x:="$x" initial_pose_y:="$y" initial_pose_a:="$yaw"
              max_speed:="${COMPETITION_MAX_SPEED:-1.00}" max_turn:="${COMPETITION_MAX_TURN:-1.00}"
              visual_max_speed:="${COMPETITION_VISUAL_MAX_SPEED:-0.30}" visual_max_turn:="${COMPETITION_VISUAL_MAX_TURN:-0.50}"
              auto_start:=false show_rviz:=false base_port:=/dev/ttyUSB0
              gimbal_port:=/dev/ttyUSB1 camera_device:=/dev/video4)
      if [[ -s "$field_dir/lidar_self_echo.yaml" ]]; then
        launch+=(lidar_self_echo_config:=/workspace/field/lidar_self_echo.yaml)
      fi
    fi
    docker run --detach --rm --init --name "$container" --network host \
      --label "competition.field=$field" --label "competition.phase=$phase" \
      --user "$(id -u):$(id -g)" "${devices[@]}" "${mounts[@]}" "${gui[@]}" \
      --mount "type=bind,src=$build_dir,dst=/workspace/compiled,readonly" \
      --mount "type=bind,src=$field_dir,dst=/workspace/field" \
      -e ROS_HOME=/workspace/field/.ros -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
      "$image" bash -c '
        source /opt/ros/noetic/setup.bash && test "$ROS_DISTRO" = noetic &&
        source /workspace/LakiBeam_ws/devel/setup.bash &&
        source /workspace/compiled/devel/setup.bash &&
        exec "$@"
      ' _ "${launch[@]}" >/dev/null
    if ! ros timeout 12 bash -c 'until rosparam get /rosdistro >/dev/null 2>&1; do sleep 0.25; done'; then
      printf 'ROS master 未就绪，请查看启动日志。docker logs %s\n' "$container" >&2
      exit 1
    fi
    printf '%s 已启动。实时日志：docker logs -f %s\n' "$phase" "$container"
    ;;
  rviz)
    require_phase mapping
    if [[ "$(docker inspect --format '{{.State.Running}}' "$rviz_container" 2>/dev/null || true)" == true ]] ||
       ros timeout 2 rosnode ping -c 1 /rviz 2>/dev/null | grep -Fq 'xmlrpc reply from'; then
      printf '本轮的 RViz 已经在运行。\n'
      exit 0
    fi
    rviz_display_options
    docker run --detach --rm --init --name "$rviz_container" --network host \
      --user "$(id -u):$(id -g)" "${gui[@]}" \
      --mount "type=bind,src=$root,dst=/workspace/xiaoche,readonly" \
      -e ROS_HOME=/tmp/competition-rviz -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
      "$image" bash -c '
        source /opt/ros/noetic/setup.bash &&
        exec rviz -d /workspace/xiaoche/src/maze_bridge/rviz/slam_toolbox_mapping.rviz __name:=rviz
      ' >/dev/null
    printf '正在为当前建图打开 RViz。\n'
    ;;
  turn)
    printf '无前置点架构已无需手动标记折返点，推回起点后直接执行 save 即可。\n'
    ;;
  save)
    require_phase mapping
    ros timeout 32 python3 /workspace/xiaoche/tools/save_competition_course.py save /workspace/field
    docker stop --signal INT --timeout 15 "$container" >/dev/null
    close_mapping_viewer
    printf '地图和起点已保存到 %s。\n' "$field_dir"
    ;;
  status)
    phase="$(docker inspect --format '{{ index .Config.Labels "competition.phase" }}' "$container" 2>/dev/null || true)"
    if [[ "$phase" == mapping ]]; then
      require_phase mapping
      ros timeout 8 rostopic echo -n 1 /map/info
    else
      require_phase navigation
      ros timeout 4 rostopic echo -n 1 /blue_autonomy/state/data
      ros timeout 4 rostopic echo -n 1 /gimbal/servo_ready/data
      ros timeout 4 rostopic echo -n 1 /amcl_pose/pose/pose
      check_navigation_scan
      printf '雷达数据正常。\n'
    fi
    ;;
  go)
    require_phase navigation
    [[ -t 0 ]] || { printf 'go 需要交互终端。\n' >&2; exit 1; }
    mkdir -p "$field_dir/bags"
    exec 9>"$field_dir/.attempt.lock"
    flock -n 9 || { printf '该场次已有 go 终端。\n' >&2; exit 1; }
    if docker inspect "$bag_container" >/dev/null 2>&1; then
      printf '已有录包容器在运行。\n' >&2
      exit 1
    fi
    attempt=1
    for previous in "$field_dir"/bags/attempt-[0-9]*; do
      [[ -d "$previous" ]] || continue
      index="${previous##*/attempt-}"
      index="${index%%-*}"
      [[ "$index" =~ ^[0-9]+$ ]] || continue
      if ((10#$index >= attempt)); then attempt=$((10#$index + 1)); fi
    done
    bag_dir="$(mktemp -d "$field_dir/bags/$(printf 'attempt-%02d' "$attempt")-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
    bag_name="${bag_dir##*/}"
    (
      timeout 20 bash -c 'until [[ -f "$1/.start-recording" ]]; do sleep 0.05; done' _ "$bag_dir" &&
        start_navigation_recording "$bag_dir"
    ) >"$bag_dir/record-start.log" 2>&1 9>&- &
    bag_starter=$!
    finish_attempt() {
      touch "$bag_dir/.stop-attempt"
      # If the recorder is starting, wait for its bounded Docker/rosbag
      # readiness check before collecting it so a late launch cannot escape.
      if [[ -f "$bag_dir/.start-recording" ]]; then
        wait "$bag_starter" || true
      else
        kill "$bag_starter" 2>/dev/null || true
        wait "$bag_starter" 2>/dev/null || true
        return 0
      fi
      bag_live="$(docker inspect --format '{{ index .Config.Labels "competition.bag" }}' "$bag_container" 2>/dev/null || true)"
      if [[ "$bag_live" == "$bag_name" ]]; then
        docker stop --signal INT --timeout 35 "$bag_container" >/dev/null
      fi
      if compgen -G "$bag_dir/*.bag" >/dev/null; then
        printf '本轮录包已保存到 %s。\n' "$bag_dir"
      else
        printf '未生成完整录包，请查看 %s。\n' "$bag_dir" >&2
        return 1
      fi
    }
    trap 'finish_attempt' EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    trap 'exit 129' HUP
    docker exec -it "$container" bash -c '
      source /opt/ros/noetic/setup.bash && test "$ROS_DISTRO" = noetic &&
      source /workspace/LakiBeam_ws/devel/setup.bash &&
      source /workspace/compiled/devel/setup.bash &&
      exec python3 /workspace/xiaoche/tools/trigger_race_go.py /workspace/field "$1"
    ' _ "/workspace/field/bags/$bag_name"
    ;;
  record)
    require_phase navigation
    exec 9>"$field_dir/.attempt.lock"
    flock -n 9 || { printf '本轮 go 已管理录包。\n' >&2; exit 1; }
    start_navigation_recording
    ;;
  stop)
    live="$(docker inspect --format '{{ index .Config.Labels "competition.field" }}' "$container" 2>/dev/null || true)"
    bag_field="$(docker inspect --format '{{ index .Config.Labels "competition.field" }}' "$bag_container" 2>/dev/null || true)"
    if [[ "$live" != "$field" && "$bag_field" != "$field" ]]; then
      printf '该场次没有需要停止的任务。\n'
      exit 0
    fi
    if [[ "$live" == "$field" ]]; then
      docker stop --signal INT --timeout 15 "$container"
      close_mapping_viewer
    fi
    if [[ "$bag_field" == "$field" ]]; then
      docker stop --signal INT --timeout 35 "$bag_container" >/dev/null
      printf 'rosbag 录制已结束，文件在 %s/bags/。\n' "$field_dir"
    fi
    ;;
esac
