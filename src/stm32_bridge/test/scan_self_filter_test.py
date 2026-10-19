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
    def test_two_streams_preserve_visibility_semantics(self):
        messages = {}
        lock = threading.Lock()
        def receive(message, topic):
            with lock:
                messages[topic] = message
        subscribers = [rospy.Subscriber(topic, LaserScan, receive, topic, queue_size=1)
                       for topic in ['/filter_test/slam', '/filter_test/obstacles']]
        publisher = rospy.Publisher('/filter_test/raw', LaserScan, queue_size=1)
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
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            publisher.publish(scan)
            time.sleep(.03)
            with lock:
                if len(messages) == 2:
                    break
        with lock:
            self.assertEqual(len(messages), 2, 'Both scan streams must publish')
            slam, obstacle = messages['/filter_test/slam'], messages['/filter_test/obstacles']
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
        for subscriber in subscribers:
            subscriber.unregister()
        publisher.unregister()


if __name__ == '__main__':
    rospy.init_node('scan_observation_contract_test')
    rostest.rosrun('stm32_bridge', 'scan_self_filter_observation_contract', ScanObservationContract)
