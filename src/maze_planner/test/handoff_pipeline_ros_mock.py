#!/usr/bin/env python3
"""Kinematic competition scenarios with real move_base, planners and handoff nodes.

Run on an isolated Noetic ROS master on port 11329. The sensors, localization
and drivetrain are simulated. Navigation consumes the production launch params.
"""
import argparse
import errno
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import tempfile
import time

import cv2
import numpy as np
import roslaunch
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger, TriggerResponse
from stm32_bridge.msg import BlueTargetObservation


MASTER = "http://127.0.0.1:11329"
RANGES = np.linspace(-.75 * math.pi, .75 * math.pi, 1081)
CAM_FOCAL = 640 / (2 * math.tan(math.radians(44)))
WORKSPACE = Path(__file__).resolve().parents[3]
START = (1.0, 0.0, 0.0)
FALSE_BLUE_FIELD = "false_blue_deadend_and_narrow_dynamic"
WALL_ANCHOR_FIELD = "wall-anchor-upgrade"


def yaw_of(quaternion):
    return 2 * math.atan2(quaternion.z, quaternion.w)


def angle(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def corners(rect):
    x0, y0, x1, y1 = rect
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], np.float32)


class Field:
    def __init__(self, name):
        self.name = name
        self.verify_dropout = name == "box"
        self.camera_range = 5.48 if name == "box" else 7.5
        if name in ("branches", "pre-start"):
            self.floors = [(0, -1.15, 6.25, 1.15), (4.00, -3.35, 6.25, 4.30)]
            self.walls = [(0, -1.15, 4.00, -1.15), (0, 1.15, 4.00, 1.15),
                          (0, -1.15, 0, 1.15), (4.00, -3.35, 4.00, -1.15),
                          (4.00, 1.15, 4.00, 4.30), (6.25, -3.35, 6.25, 4.30),
                          (4.00, -3.35, 6.25, -3.35), (4.00, 4.30, 6.25, 4.30)]
            self.attempts = [((5.125, -3.35), (0, 1), []),
                             ((5.125, 4.30), (0, -1), [(1.95, -.70, 2.45, -.20)])]
        elif name == FALSE_BLUE_FIELD:
            self.floors = [(0, -1.60, 7, 1.60), (4.40, -1.60, 7, 4.25)]
            self.walls = [(0, -1.60, 7, -1.60), (0, -1.60, 0, 1.60),
                          (0, 1.60, 4.40, 1.60), (4.40, 1.60, 4.40, 4.25),
                          (4.40, 4.25, 7, 4.25), (7, -1.60, 7, 4.25)]
            self.attempts = [((6.10, 4.25), (0, -1), [])]
            self.dynamic_box = (4.41, 2.12, 5.62, 2.65)
        else:
            self.floors = [(0, -1.60, 7.0, 1.60)]
            self.walls = [(0, -1.60, 7.0, -1.60), (0, 1.60, 7.0, 1.60),
                          (0, -1.60, 0, 1.60), (7.0, -1.60, 7.0, 1.60)]
            self.attempts = [((7., 0.), (-1, 0), [(5.35, -.25, 5.85, .25)])] if name == "box" else [
                ((7., 1.18), (-1, 0), [])]
            if name == WALL_ANCHOR_FIELD:
                self.attempts = [((7., 0.), (-1, 0), [])]

    def occupancy(self, omit_finish_wall=False):
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = rospy.Time.now()
        res = .025
        width, height = 328, 356
        ox, oy = -.60, -4.0
        grid.info.resolution, grid.info.width, grid.info.height = res, width, height
        grid.info.origin.position.x, grid.info.origin.position.y = ox, oy
        grid.info.origin.orientation.w = 1.0
        x, y = np.meshgrid(ox + res * (np.arange(width) + .5),
                           oy + res * (np.arange(height) + .5))
        free = np.zeros(x.shape, dtype=bool)
        for x0, y0, x1, y1 in self.floors:
            if omit_finish_wall:
                x1 = ox + width * res
            free |= (x > x0) & (x < x1) & (y > y0) & (y < y1)
        grid.data = np.where(free, 0, 100).flatten().astype(np.int8).tolist()
        return grid

    def cast(self, x, y, direction, extra=()):
        dx, dy = np.cos(direction), np.sin(direction)
        distance = np.full_like(direction, 12.0, dtype=float)
        with np.errstate(divide="ignore", invalid="ignore"):
            for x0, y0, x1, y1 in self.walls + list(extra):
                if x0 == x1:
                    d = (x0 - x) / dx
                    value = y + d * dy
                    hit = (value >= y0) & (value <= y1)
                else:
                    d = (y0 - y) / dy
                    value = x + d * dx
                    hit = (value >= x0) & (value <= x1)
                distance = np.where(hit & (d > 0), np.minimum(distance, d), distance)
        return distance


class RaceTerminal:
    """Run the production key listener with a PTY and a real prewarmed rosbag."""

    def __init__(self, field, attempt):
        self.directory = field / f"attempt-{attempt:02}"
        self.directory.mkdir()
        self.master, slave = pty.openpty()
        self.output = bytearray()
        self.bag = None
        os.set_blocking(self.master, False)
        try:
            self.child = subprocess.Popen(
                ["python3", str(WORKSPACE / "tools/trigger_race_go.py"), str(field),
                 str(self.directory)], stdin=slave, stdout=slave, stderr=slave,
                start_new_session=True)
        finally:
            os.close(slave)

    def read(self):
        try:
            while True:
                chunk = os.read(self.master, 4096)
                if not chunk:
                    break
                self.output.extend(chunk)
        except BlockingIOError:
            pass
        except OSError as error:
            if error.errno != errno.EIO:
                raise
        return self.output.decode(errors="replace")

    def prepare(self):
        if self.bag is None and (self.directory / ".start-recording").is_file():
            self.bag = subprocess.Popen(
                ["rosbag", "record", "--lz4", "-O", str(self.directory / "round.bag"),
                 "/cmd_vel", "/blue_autonomy/state", "/goal_reached"], stdout=subprocess.DEVNULL)
        return "[READY]" in self.read() and self.child.poll() is None

    def trigger(self, key):
        os.write(self.master, key)

    def close(self):
        if self.child.poll() is None:
            self.child.send_signal(signal.SIGINT)
        try:
            result = self.child.wait(timeout=4)
        finally:
            if self.child.poll() is None:
                self.child.kill()
                self.child.wait()
            if self.bag is not None and self.bag.poll() is None:
                self.bag.send_signal(signal.SIGINT)
                try:
                    self.bag.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self.bag.kill()
                    self.bag.wait()
            transcript = self.read()
            os.close(self.master)
            print(transcript, flush=True)
        complete = self.directory / "round.bag"
        assert result == 0 and complete.is_file() and not list(self.directory.glob("*.active")), transcript
        assert "底盘目标速度归零" in transcript, transcript
        subprocess.run(["rosbag", "info", str(complete)], check=True, stdout=subprocess.DEVNULL)


class Simulation:
    def __init__(self, field, workspace_config):
        self.field = field
        self.pose = START
        self.command = (0.0, 0.0)
        self.command_time = 0.0
        self.reached = False
        self.state = ""
        self.nav_state = ""
        self.goal_time = None
        self.reset_seen = False
        self.reset_times = []
        self.terminal_events = []
        self.phases, self.goals = [], []
        self.nav_events, self.planning_times = [], []
        self.center_requests = 0
        self.prestart_tilt = 0.0
        self.false_stage = ""
        self.ghost_after_cooldown = False
        self.false_release_time = None
        self.anchor_observations = []
        self.anchor_positions = []
        self.anchor_entrances = []
        self.anchor_reveal_time = None
        self.repeat_observation = None
        self.repeat_until = None
        self.ghost_samples_after_release = 0
        self.ghost_accepted_after_release = 0
        self.image = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
        top_noise = np.zeros((480, 640, 3), dtype=np.uint8)
        top_noise[28:52, 312:328] = [255, 0, 0]
        top_noise[454:480, 100:118] = [255, 0, 0]
        self.noise_bytes = top_noise.tobytes()
        self.top_detections = 0
        self.top_frames_observed = 0
        self.footprint = np.array(rospy.get_param("/robot_geometry/footprint"), dtype=np.float32)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.mask = rospy.Publisher("/corner_clearance_map", OccupancyGrid, queue_size=1, latch=True)
        self.scan = rospy.Publisher("/scan", LaserScan, queue_size=2)
        self.obstacles = rospy.Publisher("/scan_obstacles", LaserScan, queue_size=2)
        self.obs = rospy.Publisher("/blue_target/observation", BlueTargetObservation, queue_size=2)
        self.joint = rospy.Publisher("/gimbal/joint_states", JointState, queue_size=2)
        self.amcl = rospy.Publisher("/amcl_pose", PoseWithCovarianceStamped, queue_size=2)
        self.odom = rospy.Publisher("/odom", Odometry, queue_size=3)
        self.enable = rospy.Publisher("/blue_autonomy/enable", Bool, queue_size=4)
        self.initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.gimbal = [rospy.Publisher("/gimbal/" + topic, Bool, queue_size=1, latch=True)
                       for topic in ("connected", "servo_ready", "target_locked", "enabled")]
        self.center_service = rospy.Service("/gimbal/center", Trigger, self.on_center)
        self.tf = tf2_ros.TransformBroadcaster()
        self.static_tf = tf2_ros.StaticTransformBroadcaster()
        self.subscribers = [
            rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=4),
            rospy.Subscriber("/goal_reached", Bool, self.on_reached, queue_size=1),
            rospy.Subscriber("/blue_autonomy/state", String, self.on_state, queue_size=5),
            rospy.Subscriber("/handoff/nav_state", String, self.on_nav_state, queue_size=5),
            rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.on_goal, queue_size=5),
            rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.on_reset, queue_size=1),
            rospy.Subscriber("/blue_target/observation", BlueTargetObservation,
                             self.on_detection, queue_size=3),
            rospy.Subscriber("/blue_target/validated_observation", BlueTargetObservation,
                             self.on_validated_detection, queue_size=3),
            rospy.Subscriber("/move_base/SmoothGlobalPlanner/diagnostics", String,
                             self.on_planner, queue_size=5),
        ]
        if field.name == WALL_ANCHOR_FIELD:
            self.subscribers.extend([
                rospy.Subscriber("/blue_target/map_pose", PoseStamped, self.on_anchor_position, queue_size=5),
                rospy.Subscriber("/blue_target/approach_goal", PoseStamped, self.on_anchor_entrance, queue_size=5),
            ])
        self.map_pub.publish(self.field.occupancy(omit_finish_wall=field.name == WALL_ANCHOR_FIELD))
        self.mask.publish(self.field.occupancy())
        for pub, value in zip(self.gimbal, (True, True, False, True)):
            pub.publish(value)
        now = rospy.Time.now()
        self.static_tf.sendTransform([
            self.transform(now, "map", "odom", 0, 0, 0),
            self.transform(now, "base_footprint", "laser", .315, 0, 0),
        ])
        names = {"move_base", "handoff_blue_navigator", "handoff_direct_visual", "visual_handoff_manager"}
        if field.name == FALSE_BLUE_FIELD:
            names.add("blue_detector")
        self.children = []
        try:
            for node in workspace_config.nodes:
                if node.name not in names:
                    continue
                remaps = [f"{topic}:={destination}" for topic, destination in node.remap_args]
                self.children.append(subprocess.Popen(
                    ["rosrun", node.package, node.type, f"__name:={node.name}", *remaps]))
            assert len(self.children) == len(names)
        except BaseException:
            self.close()
            raise

    def close(self):
        for child in self.children:
            child.terminate()
        for child in self.children:
            try:
                child.wait(timeout=4)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()

    @staticmethod
    def transform(stamp, source, child, x, y, yaw):
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, source, child
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return tf

    def on_cmd(self, message):
        self.command = (message.linear.x, message.angular.z)
        self.command_time = time.monotonic()

    def on_reached(self, message):
        if message.data and not self.reached:
            self.goal_time = time.monotonic()
        self.reached = message.data

    def on_state(self, message):
        self.state = message.data
        label = message.data.partition(":")[0]
        if not self.phases or self.phases[-1] != label:
            self.phases.append(label)
        if label == "TERMINAL_PARK":
            self.terminal_events.append((message.data, tuple(self.pose)))
            if self.first_handoff is None:
                self.first_handoff = tuple(self.pose)
                self.anchor_terminal_time = time.monotonic()

    def on_nav_state(self, message):
        self.nav_state = message.data
        now = time.monotonic()
        self.nav_events.append((now, message.data))
        if "false blue target suppressed" in message.data:
            self.false_release_time = now
        if self.nav_state.startswith("OBSERVE:"):
            self.observe_positions.append(tuple(self.pose))

    def on_center(self, _request):
        self.center_requests += 1
        return TriggerResponse(success=True)

    def on_detection(self, observation):
        if observation.header.frame_id == "top_blue_noise":
            self.top_frames_observed += 1
            self.top_detections += observation.detected

    def on_anchor_position(self, goal):
        self.anchor_positions.append((time.monotonic(), goal.pose.position.x))

    def on_anchor_entrance(self, goal):
        self.anchor_entrances.append((time.monotonic(), goal.pose.position.x))

    def on_validated_detection(self, observation):
        if observation.header.frame_id == "unanchored_wall_marker":
            self.anchor_observations.append(
                (time.monotonic(), observation.header.stamp.to_nsec(), observation.detected))
        if (self.false_stage == "repeat" and observation.detected and
                observation.header.frame_id == "ghost_blue"):
            self.ghost_accepted_after_release += 1

    def on_planner(self, message):
        timing = json.loads(message.data).get("smoothing_ms")
        if timing is not None:
            self.planning_times.append(timing)

    def on_goal(self, message):
        p = message.goal.target_pose.pose.position
        self.goals.append([round(p.x, 3), round(p.y, 3)])

    def on_reset(self, message):
        p = message.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(message.pose.pose.orientation))
        self.reset_seen = True
        self.reset_times.append(time.monotonic())

    def simulate(self, step, marker, boxes, box_edges, odom_v=None, amcl_bias=0.0):
        x, y, yaw = self.pose
        v, w = self.command if time.monotonic() - self.command_time < .25 else (0.0, 0.0)
        middle = yaw + step*w/2
        x += step * v * math.cos(middle)
        y += step * v * math.sin(middle)
        yaw = angle(yaw + step * w)
        self.pose = (x, y, yaw)

        stamp = rospy.Time.now()
        tf = self.transform(stamp, "odom", "base_footprint", x, y, yaw)
        self.tf.sendTransform(tf)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, "odom", "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x = v if odom_v is None else odom_v
        odom.twist.twist.angular.z = w
        self.odom.publish(odom)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp, pose.header.frame_id = stamp, "map"
        pose.pose.pose.position.x = x + amcl_bias
        pose.pose.pose.position.y = y
        pose.pose.pose.orientation = tf.transform.rotation
        pose.pose.covariance[0] = pose.pose.covariance[7] = .02**2
        self.amcl.publish(pose)

        c, s = math.cos(yaw), math.sin(yaw)
        laser_x, laser_y = x + .315*c, y + .315*s
        distances = self.field.cast(laser_x, laser_y, yaw + RANGES, box_edges)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, "laser"
        scan.angle_min, scan.angle_max = RANGES[0], RANGES[-1]
        scan.angle_increment = RANGES[1] - RANGES[0]
        scan.range_min, scan.range_max = .10, 10.0
        scan.ranges = np.where(distances < 10.0, distances, np.inf).tolist()
        self.obstacles.publish(scan)
        if self.field.name == WALL_ANCHOR_FIELD:
            if x < 4.25:
                # The costmap can still see the boundary. The filtered scan
                # used to localize the marker loses returns from the far wall.
                hit_x = laser_x + distances * np.cos(yaw + RANGES)
                scan.ranges = np.where((distances < 10.0) & (hit_x < 6.99), distances, np.inf).tolist()
            elif self.anchor_reveal_time is None:
                self.anchor_reveal_time = time.monotonic()
        self.scan.publish(scan)

        ghost = self.false_stage in ("ghost", "repeat")
        if ghost:
            marker = (7.0, -.30)
        cam_x, cam_y = x + .145*c, y + .145*s
        dx, dy = marker[0] - cam_x, marker[1] - cam_y
        distance = math.hypot(dx, dy)
        absolute = math.atan2(dy, dx)
        bearing = angle(absolute - yaw)
        sweep = 165 if self.false_stage == "repeat" else 82
        visible = distance < self.field.camera_range and abs(bearing) <= math.radians(sweep)
        pan = max(math.radians(-135), min(math.radians(135), bearing)) if visible else 0.0
        if visible and not ghost:
            wall_range = self.field.cast(cam_x, cam_y, np.array([absolute]))[0]
            # The simulated camera can see above the box. The laser cannot.
            visible = abs(wall_range - distance) <= .08
        if self.field.verify_dropout and self.state.startswith("TERMINAL_PARK:") and x > 6.315:
            self.dropout_samples += 1
            visible = False
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = ["camera_pan_joint", "camera_tilt_joint"]
        joint.position = [pan if visible else 0.0, self.prestart_tilt]
        self.joint.publish(joint)
        obs = BlueTargetObservation()
        obs.header.stamp, obs.detected = stamp, visible
        obs.image_width, obs.image_height = 640, 480
        obs.center_x = 320 - CAM_FOCAL * math.tan(bearing - pan) if visible else 320
        obs.center_y = 240
        estimated_distance = distance
        if self.field.name == WALL_ANCHOR_FIELD and x < 4.25:
            estimated_distance -= .75
        obs.pixel_height = CAM_FOCAL * .297 / max(.12, estimated_distance)
        obs.pixel_width = obs.pixel_height * .71
        obs.contour_ratio = min(.09, obs.pixel_height * obs.pixel_width / (640 * 480))
        if self.field.name == WALL_ANCHOR_FIELD:
            obs.header.frame_id = "unanchored_wall_marker"
            now = time.monotonic()
            if self.repeat_until is None and x >= 2.7:
                self.repeat_observation = obs
                self.repeat_until = now + .26
            if self.repeat_until is not None and now < self.repeat_until:
                obs = self.repeat_observation
            self.obs.publish(obs)
        elif self.field.name != FALSE_BLUE_FIELD or self.false_stage == "finish":
            self.obs.publish(obs)
        elif ghost:
            obs.header.frame_id = "ghost_blue"
            obs.header.stamp += rospy.Duration(.017)
            obs.contour_ratio = .002
            self.obs.publish(obs)
            if obs.detected and self.false_stage == "repeat" and self.ghost_after_cooldown:
                self.ghost_samples_after_release += 1
        elif self.false_stage == "top":
            image = Image()
            image.header.stamp, image.header.frame_id = stamp, "top_blue_noise"
            image.width, image.height, image.encoding, image.step = 640, 480, "bgr8", 640 * 3
            image.data = self.noise_bytes
            self.image.publish(image)
        else:
            obs.detected = False
            self.obs.publish(obs)

        # Audit independent of the map/costmaps and motion controllers.
        body = (self.footprint @ np.array([[c, s], [-s, c]], dtype=np.float32) +
                np.array([x, y], dtype=np.float32))
        area = cv2.contourArea(body)
        inside = sum(cv2.intersectConvexConvex(body, corners(floor))[0] for floor in self.field.floors)
        if len(self.field.floors) == 2:
            a, b = self.field.floors
            overlap = (max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3]))
            inside -= cv2.intersectConvexConvex(body, corners(overlap))[0]
        hits_wall = inside < area - 1e-5
        hits_box = any(cv2.intersectConvexConvex(body, corners(box))[0] > 1e-6 for box in boxes)
        return body, hits_wall or hits_box

    def run(self, name, index, marker, normal, boxes, timeout, field_directory):
        terminal = None
        try:
            if name in ("branches", "pre-start"):
                terminal = RaceTerminal(field_directory, index)
            self.attempt(name, index, marker, normal, boxes, timeout, terminal)
        finally:
            if terminal is not None:
                terminal.close()

    def check_prestart_resets(self, terminal, marker, boxes, box_edges, rate):
        def publish(odom_v=0.0, amcl_bias=0.0):
            self.simulate(0.0, marker, boxes, box_edges, odom_v=odom_v, amcl_bias=amcl_bias)
            rate.sleep()

        def wait_for_reset(previous, timeout, amcl_bias=0.0):
            deadline = time.monotonic() + timeout
            while len(self.reset_times) == previous and time.monotonic() < deadline:
                publish(amcl_bias=amcl_bias)
            assert len(self.reset_times) == previous + 1, terminal.read()
            assert self.state.startswith("IDLE:"), self.state

        initial = len(self.reset_times)
        for _ in range(8):
            publish(.10, .45)
        terminal.trigger(b" ")
        for _ in range(5):
            publish(.10, .45)
        assert len(self.reset_times) == initial and self.state.startswith("IDLE:")
        transcript = terminal.read()
        assert "车未停稳" in transcript and "定位未收敛" in transcript, transcript

        wait_for_reset(initial, .9, .45)  # Moving then stationary permits an immediate retry.
        stopped_reset = self.reset_times[-1]
        wait_for_reset(initial + 1, 2.5, .45)  # Persistent bias triggers a second retry.
        assert self.reset_times[-1] - stopped_reset >= 1.45
        center_requests = self.center_requests
        terminal.trigger(b"r")
        wait_for_reset(initial + 2, .8)
        for _ in range(9):
            publish()
        transcript = terminal.read()
        assert transcript.count("定位偏离起点，自动复位") == 2, transcript
        assert "手动复位，已发布起点复位并清图" in transcript, transcript
        assert self.center_requests == center_requests + 1, transcript
        assert "[READY]" in transcript.rsplit("\r\033[K", 1)[-1], transcript
        print("Verified pre-start retry after motion, cooldown retry, r reset and readiness gate.",
              flush=True)

    def check_gimbal_prestart(self, terminal, marker, boxes, box_edges, rate):
        for enabled, tilt, expected in [(False, 0.0, "云台未使能(mode=0)"),
                                        (True, -13.5, "云台未平视(tilt=-13.5°)")]:
            self.gimbal[-1].publish(enabled)
            self.prestart_tilt = math.radians(tilt)
            for _ in range(8):
                self.simulate(0.0, marker, boxes, box_edges)
                rate.sleep()
            terminal.trigger(b" ")
            for _ in range(6):
                self.simulate(0.0, marker, boxes, box_edges)
                rate.sleep()
            assert expected in terminal.read(), terminal.read()
            assert self.state.startswith("IDLE:"), self.state
        self.prestart_tilt = 0.0
        print("Verified pre-start mode=0 and tilted-camera launch gates.", flush=True)

    @staticmethod
    def box_edges(boxes):
        return [edge for x0, y0, x1, y1 in boxes for edge in
                ((x0, y0, x1, y0), (x0, y1, x1, y1), (x0, y0, x0, y1), (x1, y0, x1, y1))]

    def attempt(self, name, index, marker, normal, boxes, timeout, terminal):
        self.first_handoff = None
        self.anchor_terminal_time = None
        self.dropout_samples = 0
        self.observe_positions = []
        self.nav_events.clear()
        self.planning_times.clear()
        self.phases.clear()
        self.terminal_events.clear()
        self.goals.clear()
        self.reset_seen = False
        collisions = 0
        crossings = []
        box_edges = self.box_edges(boxes)
        rate = rospy.Rate(30)
        for _ in range(70 if index == 1 else 24):
            if self.enable.get_num_connections():
                self.enable.publish(False)
            self.simulate(0.0, marker, boxes, box_edges)
            rate.sleep()
        assert self.state.startswith("IDLE:") and self.nav_state.startswith("IDLE:") and not self.reached
        if terminal is None:
            start = PoseWithCovarianceStamped()
            start.header.stamp, start.header.frame_id = rospy.Time.now(), "map"
            start.pose.pose.position.x, start.pose.pose.position.y = START[:2]
            start.pose.pose.orientation.w = 1.0
            self.initialpose.publish(start)
            for _ in range(16):
                self.simulate(0.0, marker, boxes, box_edges)
                rate.sleep()
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=5)
            rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
            assert self.reset_seen and self.pose == START
            self.enable.publish(True)
        else:
            deadline = time.monotonic() + 18
            while not terminal.prepare():
                assert time.monotonic() < deadline, terminal.read()
                assert terminal.child.poll() is None, terminal.read()
                self.simulate(0.0, marker, boxes, box_edges)
                rate.sleep()
            assert self.reset_seen and self.pose == START
            if index == 1 and name == "pre-start":
                self.check_gimbal_prestart(terminal, marker, boxes, box_edges, rate)
                self.check_prestart_resets(terminal, marker, boxes, box_edges, rate)
            terminal.trigger(b" " if index == 1 else b"\n")
        resets_before_race = len(self.reset_times)
        if name == "pre-start":
            deadline = time.monotonic() + 3.0
            while self.state.startswith("IDLE:"):
                assert time.monotonic() < deadline and terminal.child.poll() is None, terminal.read()
                self.simulate(0.0, marker, boxes, box_edges)
                rate.sleep()
            terminal.trigger(b"r")
            for _ in range(8):
                self.simulate(0.0, marker, boxes, box_edges)
                rate.sleep()
            assert len(self.reset_times) == resets_before_race
            assert "已发布发车信号" in terminal.read(), terminal.read()
            print(json.dumps(dict(scenario=name, attempt=index, launched=True,
                                  post_launch_resets=0)), flush=True)
            return
        started = last = time.monotonic()
        first_approach = None
        ghost_started = false_released = None
        dynamic_inserted = False
        narrow_clearances = []
        still = 0.0
        while time.monotonic() - started < timeout:
            now = time.monotonic()
            if name == FALSE_BLUE_FIELD:
                if 1.5 <= now - started < 2.4:
                    self.false_stage = "top"
                elif ghost_started is None:
                    self.false_stage = ""
                    if self.pose[0] >= 4.15:
                        assert not any(state.startswith("APPROACH:") for _, state in self.nav_events)
                        ghost_started = now
                        self.false_stage = "ghost"
                elif false_released is None:
                    if self.nav_state.startswith("APPROACH:"):
                        self.false_stage = "absent"
                    if self.false_release_time is not None:
                        false_released = self.false_release_time
                        self.false_stage = "repeat"
                        boxes = [self.field.dynamic_box]
                        box_edges = self.box_edges(boxes)
                        dynamic_inserted = True
                elif now - false_released < 3.4:
                    self.false_stage = "repeat"
                    self.ghost_after_cooldown = now - false_released >= 2.05
                else:
                    self.false_stage = "finish"
            body, collision = self.simulate(min(.06, now - last), marker, boxes, box_edges)
            last = now
            collisions += bool(collision)
            if name == FALSE_BLUE_FIELD and dynamic_inserted:
                _, y0, x1, y1 = self.field.dynamic_box
                if y0 <= self.pose[1] <= y1:
                    narrow_clearances.append(float(body[:, 0].min()) - x1)
            if name == "box" and 5.35 <= self.pose[0] <= 5.85:
                crossings.append(abs(self.pose[1]))
            if self.state.startswith("TARGET_APPROACH:") and first_approach is None:
                first_approach = tuple(self.pose)
                if terminal is not None:
                    terminal.trigger(b"r\n")
            if self.reached:
                if self.command != (0.0, 0.0):
                    raise AssertionError("nonzero velocity after completion")
                still = now - self.goal_time
                if still >= 3.0 and (terminal is None or "静止 3.0 s" in terminal.read()):
                    break
            rate.sleep()

        offsets = body - np.array(marker, dtype=np.float32)
        depth = offsets @ np.array(normal)
        lateral = offsets @ np.array([-normal[1], normal[0]])
        handoff_distance = (None if self.first_handoff is None else
                            math.hypot(self.first_handoff[0] - marker[0], self.first_handoff[1] - marker[1]))
        report = dict(scenario=name, attempt=index, phases=self.phases, goals=self.goals,
                      collisions=collisions, last_state=self.state, final_pose=self.pose,
                      depth=[float(min(depth)), float(max(depth))],
                      lateral=[float(min(lateral)), float(max(lateral))],
                      side_clearance=min(crossings) if crossings else None,
                      first_approach=first_approach, handoff_distance=handoff_distance,
                      observed=self.observe_positions, still_seconds=still,
                      dropout_samples=self.dropout_samples, terminal_events=self.terminal_events)
        if name == FALSE_BLUE_FIELD:
            ghost_approach = next((t for t, state in self.nav_events if state.startswith("APPROACH:")), None)
            report.update(top_detections=self.top_detections, top_frames=self.top_frames_observed,
                          ghost_verification_seconds=None if false_released is None else false_released - ghost_approach,
                          ghost_entrances=sum("entrance reached" in state and
                                             (false_released is None or t <= false_released)
                                             for t, state in self.nav_events),
                          ghost_samples_after_release=self.ghost_samples_after_release,
                          ghost_accepted_after_release=self.ghost_accepted_after_release,
                          planner_max_ms=max(self.planning_times, default=0.0),
                          dynamic_obstacle_inserted=dynamic_inserted,
                          narrow_clearance=min(narrow_clearances) if narrow_clearances else None)
        if name == WALL_ANCHOR_FIELD:
            reveal = self.anchor_reveal_time
            first_terminal = self.anchor_terminal_time
            frames = [(t, stamp, detected) for t, stamp, detected in self.anchor_observations
                      if first_terminal is None or t < first_terminal]
            times = [t for t, _, _ in frames]
            report.update(
                projected_x_before=[x for t, x in self.anchor_positions if reveal is None or t < reveal][-4:],
                projected_x_after=[x for t, x in self.anchor_positions if reveal is not None and t >= reveal][-4:],
                entrance_x=[x for _, x in self.anchor_entrances],
                validated_count=len(frames),
                rejected_count=sum(not detected for _, _, detected in frames),
                duplicate_stamps=len(frames) - len({stamp for _, stamp, _ in frames}),
                maximum_validated_gap=max((b - a for a, b in zip(times, times[1:])), default=0),
                validated_to_handoff=None if first_terminal is None or not times else first_terminal - times[-1],
            )
        print(json.dumps(report), flush=True)
        assert collisions == 0, "physical footprint touched a wall or box"
        assert len(self.reset_times) == resets_before_race, "localization reset after launch"
        assert self.reached and still >= 3.0, "terminal completion or three-second zero hold missing"
        assert 0 <= min(depth) and max(depth) <= .80 and max(abs(lateral)) <= .40, "outside finish"
        expected = ["EXPLORE", "TARGET_APPROACH", "TERMINAL_PARK", "COMPLETE"]
        assert all(label in self.phases for label in expected)
        assert [self.phases.index(label) for label in expected] == sorted(
            self.phases.index(label) for label in expected)
        assert handoff_distance is not None and handoff_distance <= 1.34
        if name == "corner":
            assert self.phases.count("TERMINAL_PARK") == 1, "corner handoff bounced back to navigation"
            assert not any("BLOCKED" in event for event, _ in self.terminal_events)
        if name == "box":
            assert 1.10 < first_approach[0] < 5.10 and crossings and min(crossings) > .45
            assert self.dropout_samples > 0
        if name == FALSE_BLUE_FIELD:
            assert self.top_frames_observed >= 8 and self.top_detections == 0, report
            assert false_released is not None and report["ghost_verification_seconds"] < 10.0, report
            assert report["ghost_entrances"] == 2, report
            assert self.ghost_samples_after_release > 5 and not self.ghost_accepted_after_release, report
            assert not any(state.startswith("APPROACH:") and false_released < t < false_released + 3.4
                           for t, state in self.nav_events)
            assert dynamic_inserted and 0.0 < report["planner_max_ms"] < 170.0, report
            assert narrow_clearances and min(narrow_clearances) > 0.0, report
        if name == WALL_ANCHOR_FIELD:
            assert report["projected_x_before"] and all(
                abs(x - 6.25) < .03 for x in report["projected_x_before"]), report
            assert report["projected_x_after"] and all(
                abs(x - 7.0) < .03 for x in report["projected_x_after"]), report
            assert any(abs(x - 5.45) < .04 for x in report["entrance_x"]), report
            assert any(abs(x - 6.20) < .04 for x in report["entrance_x"]), report
            assert report["validated_count"] > 40 and report["rejected_count"] == 0, report
            assert report["duplicate_stamps"] >= 2 and report["maximum_validated_gap"] < .20, report
            assert report["validated_to_handoff"] < .20, report
            assert self.phases[self.phases.index("TARGET_APPROACH"):] == [
                "TARGET_APPROACH", "TERMINAL_PARK", "COMPLETE"], report
        if name == "branches" and index == 1:
            assert any(position[1] > 2.5 for position in self.observe_positions)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=("box", "corner", "branches", "pre-start", FALSE_BLUE_FIELD, WALL_ANCHOR_FIELD))
    parser.add_argument("--timeout", type=float, default=65)
    args = parser.parse_args(rospy.myargv()[1:])
    if os.environ.get("ROS_MASTER_URI") != MASTER:
        parser.error("run on isolated ROS master port 11329")
    rospy.init_node("competition_handoff_sim")
    launch = WORKSPACE / "src/maze_bridge/launch/navigation_visual_handoff.launch"
    config = roslaunch.config.load_config_default([(str(launch), [
        "map_file:=/tmp/simulated_map.yaml", f"initial_pose_x:={START[0]}",
        f"initial_pose_y:={START[1]}", f"initial_pose_a:={START[2]}", "auto_start:=false",
    ])], None)
    for key, param in config.params.items():
        rospy.set_param(key, param.value)
    field = Field(args.scenario)
    sim = Simulation(field, config)
    try:
        with tempfile.TemporaryDirectory(prefix="competition-sim-") as tempdir:
            directory = Path(tempdir)
            (directory / "start_pose.json").write_text(
                json.dumps(dict(zip(("x", "y", "yaw"), START))))
            for index, (marker, normal, boxes) in enumerate(field.attempts, 1):
                sim.run(args.scenario, index, marker, normal, boxes, args.timeout, directory)
    finally:
        sim.enable.publish(False)
        time.sleep(.1)
        sim.close()


if __name__ == "__main__":
    main()
