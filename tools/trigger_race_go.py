#!/usr/bin/env python3
"""Reset, start and finish one race using the running competition ROS master."""

import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import threading
import time
import tty

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class Race:
    def __init__(self, field, recording):
        self.recording = recording
        self.stop_request = recording / ".stop-attempt"
        self.start = json.loads((field / "start_pose.json").read_text())
        if not all(math.isfinite(self.start[key]) for key in ("x", "y", "yaw")):
            raise ValueError("start_pose.json 含无效坐标。")
        self.stopping = False
        self.state = self.nav_state = ""
        self.last_display = ""
        self.last_phase = ""
        self.scan_time = 0.0
        self.joint_reading = (0.0, math.nan)
        self.odom_reading = (0.0, False, 0.0)
        self.amcl_reading = (0.0, math.inf, math.inf, math.inf)
        self.pose_reset_time = 0.0
        self.ready = self.connected = self.reached = False
        self.gimbal_enabled = self.gimbal_locked = False
        tilt_sign = float(rospy.get_param("/gimbal_bridge/tilt_to_ros_sign", -1.0))
        tilt_sign = -1.0 if tilt_sign < 0.0 else 1.0
        default_servo_tilt = float(rospy.get_param("/gimbal_bridge/default_tilt_deg", 0.0))
        max_servo_tilt = float(rospy.get_param("/gimbal_bridge/tilt_max_deg", 0.0))
        self.expected_ros_tilt_deg = tilt_sign * default_servo_tilt
        self.min_allowed_ros_tilt_deg = tilt_sign * max_servo_tilt - 2.0
        self.center_thread = None
        self.command_still = False
        self.enable = rospy.Publisher("/blue_autonomy/enable", Bool, queue_size=4, tcp_nodelay=True)
        self.initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,
                                          queue_size=1, tcp_nodelay=True)
        self.clear = rospy.ServiceProxy("/move_base/clear_costmaps", Empty, persistent=True)
        self.subscribers = [
            rospy.Subscriber("/blue_autonomy/state", String, self.on_state, queue_size=2),
            rospy.Subscriber("/handoff/nav_state", String, self.on_nav_state, queue_size=2),
            rospy.Subscriber("/goal_reached", Bool, self.on_goal, queue_size=1),
            rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=1),
            rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.on_amcl, queue_size=1),
            rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=1),
            rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1),
            rospy.Subscriber("/gimbal/joint_states", JointState, self.on_joint, queue_size=1),
            rospy.Subscriber("/gimbal/connected", Bool, self.on_connected, queue_size=1),
            rospy.Subscriber("/gimbal/servo_ready", Bool, self.on_ready, queue_size=1),
            rospy.Subscriber("/gimbal/enabled", Bool, self.on_enabled, queue_size=1),
            rospy.Subscriber("/gimbal/target_locked", Bool, self.on_locked, queue_size=1),
        ]

    def on_state(self, message):
        self.state = message.data

    def on_nav_state(self, message):
        self.nav_state = message.data

    def on_goal(self, message):
        self.reached = message.data

    def on_cmd(self, message):
        self.command_still = abs(message.linear.x) < 1e-4 and abs(message.angular.z) < 1e-4

    def on_scan(self, message):
        if any(math.isfinite(r) and message.range_min < r < message.range_max for r in message.ranges):
            self.scan_time = time.monotonic()

    def on_joint(self, message):
        joints = dict(zip(message.name, message.position))
        tilt = joints.get("camera_tilt_joint", math.nan)
        if "camera_pan_joint" in joints and math.isfinite(tilt):
            self.joint_reading = (time.monotonic(), tilt)

    def on_connected(self, message):
        self.connected = message.data

    def on_ready(self, message):
        self.ready = message.data

    def on_enabled(self, message):
        self.gimbal_enabled = message.data

    def on_locked(self, message):
        self.gimbal_locked = message.data

    def request_gimbal_center(self):
        def center():
            try:
                rospy.wait_for_service("/gimbal/center", timeout=.25)
                if self.alive():
                    rospy.ServiceProxy("/gimbal/center", Trigger)()
            except (rospy.ROSException, rospy.ServiceException):
                pass

        if self.center_thread is None or not self.center_thread.is_alive():
            self.center_thread = threading.Thread(target=center, daemon=True)
            self.center_thread.start()

    def on_odom(self, message):
        twist = message.twist.twist
        now = time.monotonic()
        previous_time, was_still, settled_at = self.odom_reading
        still = math.hypot(twist.linear.x, twist.linear.y) < .025 and abs(twist.angular.z) < .04
        if still and not was_still and now - previous_time <= .5:
            settled_at = now
        self.odom_reading = (now, still, settled_at)

    def on_amcl(self, message):
        pose = message.pose.pose
        error = math.hypot(pose.position.x - self.start["x"], pose.position.y - self.start["y"])
        delta = yaw_of(pose.orientation) - self.start["yaw"]
        yaw_error = abs(math.atan2(math.sin(delta), math.cos(delta)))
        std = math.sqrt(max(0.0, message.pose.covariance[0], message.pose.covariance[7]))
        self.amcl_reading = (time.monotonic(), error, std, yaw_error)

    def stop(self, _signum, _frame):
        self.stopping = True

    def alive(self):
        return not self.stopping and not rospy.is_shutdown() and not self.stop_request.exists()

    def await_condition(self, predicate, seconds, error):
        deadline = time.monotonic() + seconds
        while self.alive() and not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(error)
            time.sleep(.025)
        return self.alive()

    def idle(self):
        return (self.state.startswith("IDLE:") and self.nav_state.startswith("IDLE:") and
                self.command_still and not self.reached)

    def disable(self):
        # Keep the connection alive long enough for the arbiter, navigator and
        # the serial bridge to receive the reset before closing this publisher.
        deadline = time.monotonic() + .10
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.enable.publish(False)
            time.sleep(.025)

    def align_start(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.pose.position.x = self.start["x"]
        pose.pose.pose.position.y = self.start["y"]
        pose.pose.pose.orientation.z = math.sin(self.start["yaw"] / 2)
        pose.pose.pose.orientation.w = math.cos(self.start["yaw"] / 2)
        pose.pose.covariance[0] = pose.pose.covariance[7] = .05**2
        pose.pose.covariance[35] = math.radians(6)**2
        self.pose_reset_time = time.monotonic()
        self.initialpose.publish(pose)

    def reset_start(self, reason):
        self.align_start()
        self.clear()
        print(f"\r\033[K{reason}，已发布起点复位并清图。", flush=True)
        self.last_display = ""

    def retry_alignment(self):
        now = time.monotonic()
        odom_time, still, settled_at = self.odom_reading
        amcl_time, error, _, _ = self.amcl_reading
        if (self.idle() and still and now - odom_time <= .5 and
                amcl_time > self.pose_reset_time and error > .30 and
                (settled_at > self.pose_reset_time or now - self.pose_reset_time >= 1.5)):
            self.reset_start("定位偏离起点，自动复位")

    def status(self, ready_line):
        now = time.monotonic()
        scan_ok = now - self.scan_time <= .5
        joint_time, tilt_rad = self.joint_reading
        tilt_deg = math.degrees(tilt_rad)
        gimbal_issue = ""
        if not self.connected:
            gimbal_issue = "云台未连接"
        elif not self.ready:
            gimbal_issue = "云台舵机未就绪"
        elif not self.gimbal_enabled:
            gimbal_issue = "云台未使能(mode=0)"
        elif now - joint_time > 1.5:
            gimbal_issue = "云台角度无实时数据"
        elif not self.gimbal_locked and (
            abs(tilt_deg - self.expected_ros_tilt_deg) > 4.0
            or (self.expected_ros_tilt_deg > 1.0 and tilt_deg < self.min_allowed_ros_tilt_deg)
        ):
            gimbal_issue = (
                f"云台俯角未归位(当前={tilt_deg:+.1f}°, "
                f"目标={self.expected_ros_tilt_deg:+.1f}°±4°)"
            )
        gimbal_status = "--" if gimbal_issue else f"OK (enabled, 俯角={tilt_deg:+.1f}°)"
        amcl_time, amcl_error, amcl_std, yaw_error = self.amcl_reading
        localized = (amcl_time > self.pose_reset_time and amcl_error <= .30 and
                     amcl_std <= .30 and yaw_error <= .35)
        odom_time, odom_still, _ = self.odom_reading
        stationary = odom_still and now - odom_time <= .5
        missing = [reason for ok, reason in (
            (scan_ok, "雷达无有效数据"),
            (not gimbal_issue, gimbal_issue),
            (localized, "定位未收敛"),
            (stationary, "车未停稳" if now - odom_time <= .5 else "里程计无实时数据"),
            (self.idle(), "底盘未进入待命"),
        ) if not ok]
        ready = not missing
        if ready_line:
            text = (f"[{'READY' if ready else '等待'}] 雷达:{'OK' if scan_ok else '--'} | "
                    f"云台:{gimbal_status} | AMCL误差:{amcl_error:.2f}m | "
                    f"录包:{self.recording.name}")
            text += " | 敲击 [Enter / 空格] 发车" if ready else " | " + " / ".join(missing)
            text += " | [r] 复位起点并清图"
            if text != self.last_display:
                print(f"\r\033[K{text}", end="", flush=True)
                self.last_display = text
        return ready

    def run(self):
        rospy.wait_for_service("/move_base/clear_costmaps", timeout=8)
        if not self.await_condition(
                lambda: self.enable.get_num_connections() and self.initialpose.get_num_connections(),
                8, "发车或起点话题尚未连接。"):
            return
        self.disable()
        if not self.await_condition(self.idle, 3, "未确认底盘目标归零及自动任务待命。"):
            return
        self.align_start()
        self.clear()
        (self.recording / ".start-recording").touch()
        if not self.await_condition(lambda: any(self.recording.glob("*.bag.active")), 15,
                                    "本轮 rosbag 预录未就绪。"):
            return

        fd = sys.stdin.fileno()
        original = termios.tcgetattr(fd)
        fired = False
        try:
            tty.setcbreak(fd)
            termios.tcflush(fd, termios.TCIFLUSH)
            while self.alive() and not fired:
                self.status(True)
                if select.select([fd], [], [], .05)[0]:
                    key = os.read(fd, 1)
                    if not key or not self.alive():
                        return
                    if key in (b"r", b"R"):
                        if self.idle():
                            self.request_gimbal_center()
                            self.reset_start("手动复位")
                        continue
                    if key in (b" ", b"\r", b"\n") and self.status(False):
                        start = time.monotonic()
                        self.clear()
                        if not self.alive():
                            break
                        if self.status(False):
                            self.enable.publish(True)
                            fired = True
                            elapsed = 1000 * (time.monotonic() - start)
                            print(f"\r\033[K已发布发车信号，清图与发布耗时 {elapsed:.1f} ms。", flush=True)
                        continue
                self.retry_alignment()
        finally:
            termios.tcsetattr(fd, termios.TCSANOW, original)
            if not fired:
                print()
        if not fired:
            return

        still_since = None
        reported = 0
        while self.alive():
            phase = self.state.partition(":")[0]
            if phase and phase != self.last_phase:
                print(self.state, flush=True)
                self.last_phase = phase
            now = time.monotonic()
            odom_time, odom_still, _ = self.odom_reading
            if (phase == "COMPLETE" and self.reached and self.command_still and odom_still and
                    now - odom_time <= .5):
                if still_since is None:
                    still_since = now
                elapsed = min(3, int(now - still_since))
                if elapsed > reported:
                    reported = elapsed
                    text = "，已满足静止计时，可按 Ctrl+C 结束本轮。" if elapsed == 3 else ""
                    print(f"静止 {elapsed:.1f} s{text}", flush=True)
            else:
                still_since = None
                reported = 0
            time.sleep(.05)

    def shutdown(self):
        self.disable()
        deadline = time.monotonic() + .60
        while not self.idle() and time.monotonic() < deadline and not rospy.is_shutdown():
            self.enable.publish(False)
            time.sleep(.05)
        if not self.idle():
            print("本轮收尾未收到 IDLE 和零速确认。", file=sys.stderr)
            return False
        print("已回到 IDLE，底盘目标速度归零。", flush=True)
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("field", type=Path)
    parser.add_argument("recording", type=Path)
    args = parser.parse_args(rospy.myargv()[1:])
    if not sys.stdin.isatty():
        parser.error("go 需要交互终端。")

    rospy.init_node("trigger_race_go", anonymous=True, disable_signals=True)
    race = Race(args.field, args.recording)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, race.stop)
    status = 0
    try:
        race.run()
    except (RuntimeError, OSError, rospy.ROSException, rospy.ServiceException) as error:
        print(str(error), file=sys.stderr)
        status = 1
    finally:
        if not race.shutdown():
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
