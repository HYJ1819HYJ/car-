#!/usr/bin/env python3
"""Save the static map and a validated start pose."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import tempfile

from PIL import Image
import yaml


def validate_start(directory, x, y, clearance=0.25):
    meta = yaml.safe_load((directory / "map.yaml").read_text())
    resolution = float(meta["resolution"])
    ox, oy, yaw = meta["origin"]
    dx, dy = x - ox, y - oy
    gx = (math.cos(yaw) * dx + math.sin(yaw) * dy) / resolution
    gy = (-math.sin(yaw) * dx + math.cos(yaw) * dy) / resolution
    radius = clearance / resolution

    with Image.open(directory / meta["image"]) as image:
        pixels = image.convert("L")
        for iy in range(math.floor(gy - radius), math.floor(gy + radius) + 1):
            for ix in range(math.floor(gx - radius), math.floor(gx + radius) + 1):
                # Check all cells touched by the clearance disk, including
                # cells whose centers are outside but whose edges intersect it.
                cx = gx - min(max(gx, ix), ix + 1)
                cy = gy - min(max(gy, iy), iy + 1)
                if math.hypot(cx, cy) > radius:
                    continue
                if not (0 <= ix < pixels.width and 0 <= iy < pixels.height):
                    raise ValueError("起点的 0.25 m 净空超出地图范围。")
                shade = pixels.getpixel((ix, pixels.height - iy - 1))
                occupancy = (shade if meta["negate"] else 255 - shade) / 255
                if occupancy >= meta["free_thresh"]:
                    raise ValueError("起点周围 0.25 m 内存在障碍或未建图栅格。")


def save(field, export_posegraph):
    import rospy
    import tf2_ros

    rospy.init_node("save_competition_course", anonymous=True)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    with tempfile.TemporaryDirectory(dir=field) as staging:
        directory = Path(staging)
        subprocess.run(["rosrun", "map_server", "map_saver", "-f", "map", "map:=/map"],
                       cwd=directory, timeout=20, check=True)
        pose = buffer.lookup_transform(
            "map", "base_footprint", rospy.Time(0), rospy.Duration(6)
        ).transform
        p, q = pose.translation, pose.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        if not all(map(math.isfinite, (p.x, p.y, yaw))):
            raise ValueError("起点 TF 含无效坐标。")
        validate_start(directory, p.x, p.y)
        (directory / "start_pose.json").write_text(
            json.dumps({"x": p.x, "y": p.y, "yaw": yaw}, indent=2) + "\n"
        )
        for name in ("map.pgm", "map.yaml", "start_pose.json"):
            (directory / name).replace(field / name)
    listener.unregister()

    if export_posegraph:
        from slam_toolbox_msgs.srv import SerializePoseGraph

        rospy.wait_for_service("/slam_toolbox/serialize_map", timeout=6)
        serialize = rospy.ServiceProxy("/slam_toolbox/serialize_map", SerializePoseGraph)
        response = serialize(str(field / "posegraph"))
        if response.result != 0:
            raise ValueError(f"位姿图导出失败，响应码 {response.result}。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("save",))
    parser.add_argument("field", type=Path)
    parser.add_argument("--export-posegraph", action="store_true")
    args = parser.parse_args()
    save(args.field.resolve(), args.export_posegraph)
