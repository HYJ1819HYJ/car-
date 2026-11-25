#!/usr/bin/env python3
"""Save the starting transform and outbound waypoints from a live mapping run."""

import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile


def corrected_scans(output):
    scans = []
    for row in csv.reader(output.splitlines()):
        if len(row) == 4 and row[0].isdigit():
            x, y = map(float, row[2:])
            if math.isfinite(x) and math.isfinite(y):
                scans.append((int(row[0]), x, y))
    return sorted(scans)


def outbound_waypoints(scans, end_id, start, spacing=0.80):
    scans = [scan for scan in scans if scan[0] <= end_id]
    if not scans:
        raise ValueError("折返点前没有可用的 SLAM 轨迹。")
    last = start
    waypoints = []
    for _, x, y in scans:
        point = (x, y)
        if math.dist(last, point) >= spacing:
            waypoints.append(point)
            last = point
    end = scans[-1][1:]
    if not waypoints:
        raise ValueError("折返点前的建图轨迹太短，请确认车经过了赛道。")
    if math.dist(waypoints[-1], end) < 0.20:
        waypoints[-1] = end
    else:
        waypoints.append(end)
    return waypoints


def load_scans(graph):
    output = subprocess.check_output(
        ["/workspace/compiled/competition_posegraph", str(graph.with_suffix(".posegraph"))],
        text=True, timeout=12,
    )
    scans = corrected_scans(output)
    if not scans:
        raise ValueError("位姿图中没有扫描位姿。")
    return scans


def write_course(field, points):
    route = "course_waypoints:\n" + "".join(f"  - [{x:.4f}, {y:.4f}]\n" for x, y in points)
    (field / "course_waypoints.yaml").write_text(route)
    print(route, end="")


def main(action, field):
    import rospy
    import tf2_ros
    from slam_toolbox_msgs.srv import SerializePoseGraph

    rospy.init_node("save_competition_course", anonymous=True)
    rospy.wait_for_service("/slam_toolbox/serialize_map", timeout=8)
    serialize = rospy.ServiceProxy("/slam_toolbox/serialize_map", SerializePoseGraph)
    marker = field / "outbound_end.id"
    if action == "turn":
        # Serialized scans retain their IDs even when scan timestamps are lost.
        with tempfile.TemporaryDirectory(dir=field) as directory:
            graph = Path(directory) / "turn"
            serialize(str(graph))
            marker.write_text(f"{load_scans(graph)[-1][0]}\n")
        return

    end_id = int(marker.read_text())
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    pose = buffer.lookup_transform(
        "map", "base_footprint", rospy.Time(0), rospy.Duration(6)
    ).transform
    p, q = pose.translation, pose.rotation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    graph = field / "posegraph"
    serialize(str(graph))
    points = outbound_waypoints(load_scans(graph), end_id, (p.x, p.y))
    (field / "start_pose.json").write_text(
        json.dumps({"x": p.x, "y": p.y, "yaw": yaw}, indent=2) + "\n"
    )
    write_course(field, points)


if __name__ == "__main__":
    main(sys.argv[1], Path(sys.argv[2]))
