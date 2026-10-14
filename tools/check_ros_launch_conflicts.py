#!/usr/bin/env python3
"""Reject overlapping launches that own the same chassis, lidar and TF."""

from pathlib import Path
import sys


SHARED_LAUNCHES = {
    "slam_toolbox_build_map.launch",
    "gmapping_build_map.launch",
    "navigation_stack.launch",
    "odom_calibration.launch",
}


def main(arguments):
    if any(flag in arguments for flag in ("--dump-params", "--nodes", "--files", "--args", "--find-node", "--help")):
        return
    if len(arguments) < 3 or arguments[:2] != ["roslaunch", "maze_bridge"]:
        return
    if arguments[2] not in SHARED_LAUNCHES:
        return
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            argv = (process / "cmdline").read_bytes().decode().split("\0")
        except (OSError, UnicodeError):
            continue
        for index, word in enumerate(argv):
            if word == "/opt/ros/noetic/bin/roslaunch":
                rest = argv[index + 1:]
                if len(rest) >= 2 and rest[0] == "maze_bridge" and rest[1] in SHARED_LAUNCHES:
                    print(
                        f"已有 {rest[1]} 正在运行（PID {process.name}）。"
                        "请退出原启动终端并等待整套节点退出；若正在建图，请先保存地图。",
                        file=sys.stderr,
                    )
                    raise SystemExit(1)


if __name__ == "__main__":
    main(sys.argv[1:])
