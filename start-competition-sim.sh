#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python3 - <<'PY'
import socket
with socket.socket() as connection:
    connection.settimeout(0.5)
    if connection.connect_ex(('127.0.0.1',11329)) == 0:
        raise SystemExit('比赛仿真端口 11329 已占用。请先在原仿真终端按 Ctrl+C 退出，避免重复启动。')
PY
# Dedicated master plus a simulation-only velocity topic. No hardware launch.
exec "$project_dir/../xiaoche-ros1.sh" env ROS_MASTER_URI=http://127.0.0.1:11329 \
  roslaunch --port 11329 competition_sim navigation.launch "$@"
