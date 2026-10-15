#!/usr/bin/env python3
"""Request laser localization updates while odometry confirms a stationary robot."""
import math
import threading
from collections import deque

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class StationaryUpdates:
    def __init__(self):
        self.lock = threading.Lock()
        self.samples = deque()
        self.scan_stamp = None
        self.command_stamp = None
        self.command_moving = False
        self.window = rospy.get_param("~stationary_window", 0.7)
        self.timeout = rospy.get_param("~sensor_timeout", 0.3)
        self.request = rospy.ServiceProxy("/request_nomotion_update", Empty)
        self.subscribers = [
            rospy.Subscriber("/odom", Odometry, self.odom, queue_size=20),
            rospy.Subscriber("/scan", LaserScan, self.scan, queue_size=1),
            rospy.Subscriber("/cmd_vel", Twist, self.command, queue_size=1),
        ]
        self.timer = rospy.Timer(rospy.Duration(1.0), self.tick)

    def odom(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        values = (p.x, p.y, yaw, v.linear.x, v.linear.y, v.angular.z)
        moving = (not all(math.isfinite(x) for x in values) or
                  math.hypot(v.linear.x, v.linear.y) > 0.01 or abs(v.angular.z) > 0.02)
        with self.lock:
            if self.samples and msg.header.stamp <= self.samples[-1][0]:
                self.samples.clear()
            self.samples.append((msg.header.stamp, p.x, p.y, yaw, moving))
            while len(self.samples) > 1 and (msg.header.stamp - self.samples[1][0]).to_sec() > self.window:
                self.samples.popleft()

    def scan(self, msg):
        if not any(math.isfinite(r) and msg.range_min < r < msg.range_max for r in msg.ranges):
            return
        with self.lock:
            self.scan_stamp = msg.header.stamp

    def command(self, msg):
        with self.lock:
            self.command_stamp = rospy.Time.now()
            self.command_moving = (not all(math.isfinite(v) for v in
                (msg.linear.x, msg.linear.y, msg.angular.z)) or
                abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z) > 0.001)

    def tick(self, _event):
        now = rospy.Time.now()
        with self.lock:
            if self.scan_stamp is None or len(self.samples) < 2:
                return
            first, last = self.samples[0], self.samples[-1]
            ages = [(now - stamp).to_sec() for stamp in (self.scan_stamp, last[0])]
            if any(age < 0 or age > self.timeout for age in ages):
                return
            if (last[0] - first[0]).to_sec() < self.window:
                return
            if self.command_stamp is not None and self.command_moving and (now-self.command_stamp).to_sec() < 0.5:
                return
            for _, x, y, yaw, moving in self.samples:
                angle = math.atan2(math.sin(yaw-first[3]), math.cos(yaw-first[3]))
                if moving or math.hypot(x-first[1], y-first[2]) > 0.01 or abs(angle) > 0.02:
                    return
        try:
            self.request()
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(10, "Stationary AMCL update unavailable: %s", error)


if __name__ == "__main__":
    rospy.init_node("stationary_amcl_update")
    node = StationaryUpdates()
    rospy.spin()
