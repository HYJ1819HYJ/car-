#!/usr/bin/env python3
"""A chassis shadow must never become free-space evidence for the costmap."""
import math
import threading
import time
import unittest
import rospy
import rostest
from sensor_msgs.msg import LaserScan


class ScanObservationContract(unittest.TestCase):
    def collect_scan_streams(self, scan):
        messages = {}
        lock = threading.Lock()
        def receive(message, topic):
            with lock:
                messages[topic] = message
        subscribers = [rospy.Subscriber(topic, LaserScan, receive, topic, queue_size=1)
                       for topic in ['/filter_test/slam', '/filter_test/obstacles']]
        publisher = rospy.Publisher('/filter_test/raw', LaserScan, queue_size=1)
        try:
            deadline = time.monotonic()+5
            while time.monotonic() < deadline:
                publisher.publish(scan)
                time.sleep(.03)
                with lock:
                    if len(messages) == 2:
                        return messages['/filter_test/slam'], messages['/filter_test/obstacles']
            self.fail('Both scan streams must publish')
        finally:
            for subscriber in subscribers:
                subscriber.unregister()
            publisher.unregister()

    def test_two_streams_preserve_visibility_semantics(self):
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = rospy.Time.now()
        scan.angle_min = 0.
        scan.angle_max = 2*math.pi
        scan.angle_increment = math.pi/2
        scan.time_increment = .001
        scan.scan_time = .1
        scan.range_min, scan.range_max = .05, 10.
        # External return, real no-hit, chassis echo, already invalid ray,
        # and an external return close to the front edge of the body.
        scan.ranges = [.8, float('inf'), .12, float('nan'), .06]
        scan.intensities = [10., 20., 30., 40., 50.]
        slam, obstacle = self.collect_scan_streams(scan)
        for message in [slam, obstacle]:
            self.assertEqual(message.header.frame_id, scan.header.frame_id)
            self.assertEqual(message.header.stamp, scan.header.stamp)
            self.assertAlmostEqual(message.angle_increment, scan.angle_increment, places=6)
            self.assertAlmostEqual(message.scan_time, scan.scan_time, places=6)
            self.assertAlmostEqual(message.ranges[0], .8, places=6)
            self.assertTrue(math.isinf(message.ranges[1]) and message.ranges[1] > 0)
            self.assertTrue(math.isnan(message.ranges[3]))
            self.assertAlmostEqual(message.ranges[4], .06, places=6)
            self.assertEqual(message.intensities[0], 10.)
            self.assertEqual(message.intensities[2], 0.)
        self.assertTrue(math.isinf(slam.ranges[2]))
        self.assertTrue(math.isnan(obstacle.ranges[2]), 'An occluded ray cannot clear space')

    def test_fixed_echo_keeps_other_returns_from_the_rear(self):
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = rospy.Time.now()
        scan.angle_min = math.radians(125.5)
        scan.angle_increment = math.radians(.25)
        scan.angle_max = math.radians(127.0)
        scan.range_min, scan.range_max = .10, 10.0
        # Keep a similar-distance return at another angle and both distinct
        # ranges in the calibrated direction. Only the known echo is occluded.
        scan.ranges = [.357, .357, .334, .357, .40, .357, .357]
        slam, obstacles = self.collect_scan_streams(scan)
        self.assertTrue(math.isinf(slam.ranges[3]))
        self.assertTrue(math.isnan(obstacles.ranges[3]))
        for i in (0, 2, 4, 6):
            for stream in (slam, obstacles):
                self.assertAlmostEqual(stream.ranges[i], scan.ranges[i], places=6)


if __name__ == '__main__':
    rospy.init_node('scan_observation_contract_test')
    rostest.rosrun('stm32_bridge', 'scan_self_filter_observation_contract', ScanObservationContract)
