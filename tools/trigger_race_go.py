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
import time
import tty

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


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
        self.scan_time = self.joint_time = self.odom_time = self.amcl_time = 0.0
        self.pose_reset_time = 0.0
        self.ready = self.connected = self.reached = False
        self.odom_still = self.command_still = False
        self.amcl_error = self.amcl_std = self.amcl_yaw_error = math.inf
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
        if "camera_pan_joint" in message.name:
            self.joint_time = time.monotonic()

    def on_connected(self, message):
        self.connected = message.data

    def on_ready(self, message):
        self.ready = message.data

    def on_odom(self, message):
        twist = message.twist.twist
        self.odom_still = abs(twist.linear.x) < .025 and abs(twist.angular.z) < .04
        self.odom_time = time.monotonic()

    def on_amcl(self, message):
        pose = message.pose.pose
        self.amcl_error = math.hypot(pose.position.x - self.start["x"],
                                     pose.position.y - self.start["y"])
        delta = yaw_of(pose.orientation) - self.start["yaw"]
        self.amcl_yaw_error = abs(math.atan2(math.sin(delta), math.cos(delta)))
        self.amcl_std = math.sqrt(max(0.0, message.pose.covariance[0], message.pose.covariance[7]))
        self.amcl_time = time.monotonic()

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

    def status(self, ready_line):
        now = time.monotonic()
        scan_ok = now - self.scan_time <= .5
        gimbal_ok = self.connected and self.ready and now - self.joint_time <= 1.5
        localized = (self.amcl_time > self.pose_reset_time and self.amcl_error <= .30 and
                     self.amcl_std <= .30 and self.amcl_yaw_error <= .35)
        stationary = self.odom_still and now - self.odom_time <= .5
        ready = scan_ok and gimbal_ok and localized and stationary and self.idle()
        if ready_line:
            text = (f"[{'READY' if ready else '等待'}] 雷达:{'OK' if scan_ok else '--'} | "
                    f"云台:{'OK' if gimbal_ok else '--'} | AMCL误差:{self.amcl_error:.2f}m | "
                    f"录包:{self.recording.name}")
            if ready:
                text += " | 敲击 [Enter / 空格] 发车"
            if text != self.last_display:
                print(f"\r\033[K{text}", end="", flush=True)
                self.last_display = text
        return ready, stationary

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
        realigned = False
        fired = False
        try:
            tty.setcbreak(fd)
            termios.tcflush(fd, termios.TCIFLUSH)
            while self.alive() and not fired:
                ready, stationary = self.status(True)
                if (stationary and not realigned and self.amcl_time > self.pose_reset_time and
                        self.amcl_error > .30 and time.monotonic() - self.pose_reset_time > .5):
                    self.align_start()
                    realigned = True
                    ready = False
                if not select.select([fd], [], [], .05)[0]:
                    continue
                key = os.read(fd, 1)
                if not key:
                    return
                if key in (b" ", b"\r", b"\n") and ready and self.status(False)[0] and self.alive():
                    start = time.monotonic()
                    self.clear()
                    if not self.alive():
                        break
                    self.enable.publish(True)
                    fired = True
                    elapsed = 1000 * (time.monotonic() - start)
                    print(f"\r\033[K已发布发车信号，清图与发布耗时 {elapsed:.1f} ms。", flush=True)
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
            if (phase == "COMPLETE" and self.reached and self.command_still and self.odom_still and
                    now - self.odom_time <= .5):
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
