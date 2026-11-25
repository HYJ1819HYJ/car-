#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
rosctl="/home/asuka/Documents/Codex/2026-10-02/24-04-ros1-ros2-cubemx-clino/outputs/ros-docker/rosctl"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  printf '用法：./capture-blue-target.sh\n'
  printf '保持现有 ROS1 摄像头运行；按回车截图，输入 q 后回车退出。\n'
  printf '结果目录：%s/outputs/blue-target-captures\n' "$project_dir"
  exit 0
fi
if [[ $# -ne 0 ]]; then
  printf '不需要参数；使用 --help 查看说明。\n' >&2
  exit 2
fi
if [[ ! -x "$rosctl" ]]; then
  printf '找不到 ROS Docker 工具：%s\n' "$rosctl" >&2
  exit 1
fi
if [[ "$(docker inspect -f '{{.State.Running}}' asuka-ros1-noetic 2>/dev/null || true)" != "true" ]]; then
  printf 'ROS1 容器未运行，请先启动现场使用的 ROS1 环境。\n' >&2
  exit 1
fi

printf '照片、标注图和数据保存到：%s/outputs/blue-target-captures\n' "$project_dir"
exec "$rosctl" exec1 bash -c '
  source /opt/ros/noetic/setup.bash
  test "$ROS_DISTRO" = noetic || exit 1
  source /workspace/xiaoche/devel/setup.bash
  export TZ=Asia/Shanghai
  exec python3 /workspace/xiaoche/tools/capture_blue_target.py
'
