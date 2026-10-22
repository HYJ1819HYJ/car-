#!/usr/bin/env python3
"""Exercise the virtual gate without Stage or any hardware nodes."""
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class GateLimitsTest(unittest.TestCase):
    def setUp(self):
        self.actual = None
        self.reason = None
        self.command = rospy.Publisher('/gate_test/request', Twist, queue_size=1)
        self.scan = rospy.Publisher('/gate_test/scan', LaserScan, queue_size=1)
        self.output_sub = rospy.Subscriber('/gate_test/output', Twist, self.output)
        self.reason_sub = rospy.Subscriber('/simulation/safety_reason', String, self.state)

    def output(self, message):
        self.actual = (message.linear.x, message.angular.z)

    def state(self, message):
        self.reason = message.data

    def check_output(self, linear, angular, expected, reason, obstacle=None, stale=False):
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.angle_increment = 0.01
        scan.range_min, scan.range_max = 0.05, 10.0
        scan.ranges = [float('inf') if obstacle is None else obstacle - 0.315]
        # Ignore transient output from the preceding case and require a
        # sustained matching result while fresh inputs continue to arrive.
        stable_since = None
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            scan.header.stamp = rospy.Time.now() - rospy.Duration(1.0 if stale else 0.0)
            self.scan.publish(scan)
            self.command.publish(command)
            time.sleep(0.04)
            matches = (self.actual is not None and self.reason == reason and
                       all(abs(a-b) < 1e-6 for a, b in zip(self.actual, expected)))
            if matches:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since > 0.15:
                    return
            else:
                stable_since = None
        self.fail('Expected {} / {}, got {} / {}'.format(expected, reason, self.actual, self.reason))

    def test_speed_limits_preserve_collision_and_timeout_protection(self):
        self.check_output(3.0, 1.0, (3.0, 1.0), 'clear')
        self.check_output(6.0, 1.0, (3.0, 0.5), 'clear')
        self.check_output(3.0, 2.0, (1.5, 1.0), 'clear')
        # This return was outside the old 2 m point filter but lies inside
        # the 2.4 m straight prediction at 3 m/s.
        self.check_output(3.0, 0.0, (0.0, 0.0), 'footprint_prediction_stop', obstacle=2.2)
        self.check_output(-0.1, 0.0, (0.0, 0.0), 'navigation_reverse_stop')
        self.check_output(3.0, 1.0, (0.0, 0.0), 'lidar_timeout_stop', stale=True)


if __name__ == '__main__':
    rospy.init_node('simulation_gate_limits_test')
    rostest.rosrun('competition_sim', 'simulation_gate_velocity_contract', GateLimitsTest)
