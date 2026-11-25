#!/usr/bin/env python3
"""Exercise direct control on isolated ROS master 11329 without hardware."""
import math
import os
import subprocess
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from stm32_bridge.msg import BlueTargetObservation

if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11329':
    raise SystemExit('This mock only runs on isolated ROS master port 11329')
rospy.init_node('direct_visual_offline_mock')
rospy.set_param('/direct_visual_controller/auto_start', True)
rospy.set_param('/direct_visual_controller/require_gimbal', True)
rospy.set_param('/robot_geometry/footprint', [[.3483,.245],[.3483,-.245],
                                              [-.1517,-.245],[-.1517,.245]])
obs_pub = rospy.Publisher('/blue_target/observation', BlueTargetObservation, queue_size=2)
scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=2)
joint_pub = rospy.Publisher('/gimbal/joint_states', JointState, queue_size=2)
connected_pub = rospy.Publisher('/gimbal/connected', Bool, queue_size=1, latch=True)
ready_pub = rospy.Publisher('/gimbal/servo_ready', Bool, queue_size=1, latch=True)
commands, states, done = [], [], []
rospy.Subscriber('/cmd_vel', Twist, lambda m: commands.append((time.monotonic(),m.linear.x,m.angular.z)), queue_size=50)
rospy.Subscriber('/blue_autonomy/state', String, lambda m: states.append((time.monotonic(),m.data)), queue_size=20)
rospy.Subscriber('/goal_reached', Bool, lambda m: done.append((time.monotonic(),m.data)), queue_size=10)
connected_pub.publish(True)
ready_pub.publish(True)
proc = subprocess.Popen(['rosrun','maze_planner','direct_visual_controller',
                         '__name:=direct_visual_controller'])

def run_phase(detected, pan, area, obstacle=False, duration=.9):
    start = time.monotonic()
    rate = rospy.Rate(20)
    while time.monotonic()-start < duration and not rospy.is_shutdown():
        stamp = rospy.Time.now()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser'
        scan.angle_min = -1.5
        scan.angle_increment = .01
        scan.angle_max = 1.5
        scan.range_min = .1
        scan.range_max = 5.0
        scan.ranges = [float('inf')]*301
        if obstacle:
            scan.ranges[170] = .5  # A return left of the centerline.
        scan_pub.publish(scan)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['camera_pan_joint','camera_tilt_joint']
        joints.position = [pan,0.0]
        joint_pub.publish(joints)
        obs = BlueTargetObservation()
        obs.header.stamp = stamp
        obs.detected = detected
        obs.image_width, obs.image_height = 640, 480
        obs.center_x, obs.center_y = 320, 240
        obs.pixel_width, obs.pixel_height = 18, 28
        obs.contour_ratio = area
        obs_pub.publish(obs)
        rate.sleep()
    return start, time.monotonic()

try:
    search = run_phase(False,0.0,0.0)
    turn = run_phase(True,-.8,.02)
    follow = run_phase(True,0.0,.02)
    avoid = run_phase(True,0.0,.02,obstacle=True)
    align = run_phase(True,.15,.16)
    finish = run_phase(True,0.0,.16)
finally:
    proc.terminate()
    try:
        proc.wait(timeout=4)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()

def sampled(interval):
    return [(v,w) for t,v,w in commands if interval[0]+.3 < t < interval[1]]

print('states:', [s for _,s in states])
print('phase commands:', {name: sampled(interval)[-3:] for name,interval in
                         [('search',search),('turn',turn),('follow',follow),('avoid',avoid),('align',align),('finish',finish)]})
print('goal_reached:',done)
assert sampled(search) and all(v == 0 and w == 0 for v,w in sampled(search)),     'Base rotated during the initial gimbal-only search'
assert any(v == 0 and w < 0 for v,w in sampled(turn)), 'Right-side target did not turn right'
assert any(v > 0 and abs(w) < .01 for v,w in sampled(follow)), 'Centered target did not drive forward'
assert any(v >= 0 and w < -.1 for v,w in sampled(avoid)), 'Left obstacle did not steer away'
assert any(v == 0 and w > 0 for v,w in sampled(align)), 'Enough area did not wait for alignment'
assert not any(v and t < finish[0] for t,v in done), 'Goal latched before alignment'
assert any(v and t >= finish[0] for t,v in done), '16% aligned target did not latch stop'
finish_commands = [(v,w) for t,v,w in commands if finish[0] <= t < finish[1]]
assert any(abs(v) < 1e-9 and abs(w) < 1e-9 for v,w in finish_commands), 'Stop did not publish zero'
print('PASS: mapless search, target direction, obstacle avoidance and 16% aligned stop')
rospy.signal_shutdown('direct visual offline mock complete')
