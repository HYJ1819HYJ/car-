#!/usr/bin/env python3
"""Exercise handoff and command ownership on isolated ROS master 11329."""
import math
import os
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from stm32_bridge.msg import BlueTargetObservation

if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11329':
    raise SystemExit('This mock only runs on isolated ROS master port 11329')
rospy.init_node('visual_handoff_offline_mock')
oblique_case = sys.argv[1] if len(sys.argv) > 1 else None
if oblique_case not in (None, 'oblique-entrance', 'oblique-stalled'):
    raise SystemExit(f'Unknown handoff test case: {oblique_case}')
rospy.set_param('/visual_handoff_manager/auto_start', True)
rospy.set_param('/visual_handoff_manager/competition_mode', bool(oblique_case))
rospy.set_param('/visual_handoff_manager/required_clear_scans', 3)
rospy.set_param('/robot_geometry/footprint', [[.3483,.245],[.3483,-.245],
                                              [-.1517,-.245],[-.1517,.245]])
obs_pub = rospy.Publisher('/blue_target/observation', BlueTargetObservation, queue_size=2)
scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=2)
joint_pub = rospy.Publisher('/gimbal/joint_states', JointState, queue_size=2)
nav_pub = rospy.Publisher('/handoff/cmd_vel_nav', Twist, queue_size=2)
nav_state_pub = rospy.Publisher('/handoff/nav_state', String, queue_size=2, latch=True)
visual_pub = rospy.Publisher('/handoff/cmd_vel_visual', Twist, queue_size=2)
visual_goal_pub = rospy.Publisher('/handoff/visual_goal_reached', Bool, queue_size=2)
commands, states, nav_enable, visual_enable, goals = [], [], [], [], []
rospy.Subscriber('/cmd_vel', Twist, lambda m: commands.append((time.monotonic(),m.linear.x,m.angular.z)), queue_size=50)
rospy.Subscriber('/blue_autonomy/state', String, lambda m: states.append((time.monotonic(),m.data)), queue_size=30)
rospy.Subscriber('/handoff/nav_enable', Bool, lambda m: nav_enable.append((time.monotonic(),m.data)), queue_size=5)
rospy.Subscriber('/handoff/visual_enable', Bool, lambda m: visual_enable.append((time.monotonic(),m.data)), queue_size=5)
rospy.Subscriber('/goal_reached', Bool, lambda m: goals.append((time.monotonic(),m.data)), queue_size=5)
proc = subprocess.Popen(['rosrun','maze_planner','visual_handoff_manager',
                         '__name:=visual_handoff_manager'])

def run_phase(detected, obstacle=False, foreground_wall=False, wall_x=1.15,
              area=.02, goal=False, duration=1.3, wall_angle=0.0, nav_state=None):
    start = time.monotonic()
    rate = rospy.Rate(20)
    bearing = .40 if wall_angle else 0.0
    marker_x, marker_y = wall_x * math.cos(bearing), wall_x * math.sin(bearing)
    camera_bearing = math.atan2(marker_y, marker_x - .145)
    wall_depth = ((marker_x - .315)*math.cos(wall_angle) + marker_y*math.sin(wall_angle))
    while time.monotonic()-start < duration and not rospy.is_shutdown():
        stamp = rospy.Time.now()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_increment = 2*math.pi/1440
        scan.angle_max = math.pi-scan.angle_increment
        scan.range_min = .10
        scan.range_max = 10.0
        scan.ranges = [6.0]*1440
        for i in range(1440):
            a = scan.angle_min+i*scan.angle_increment
            projection = math.cos(a-wall_angle)
            if projection > .2:
                scan.ranges[i] = (.50-.315 if foreground_wall else wall_depth)/projection
        if obstacle:
            obstacle_x = .65 * math.cos(bearing) - .315
            obstacle_y = (.65 * math.sin(bearing)) if wall_angle else .11
            a = math.atan2(obstacle_y, obstacle_x)
            beam = round((a-scan.angle_min)/scan.angle_increment)
            for i in range(beam-4,beam+5):
                scan.ranges[i] = math.hypot(obstacle_x, obstacle_y)
        scan_pub.publish(scan)
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = ['camera_pan_joint','camera_tilt_joint']
        joint.position = [camera_bearing,0.0]
        joint_pub.publish(joint)
        obs = BlueTargetObservation()
        obs.header.stamp = stamp
        obs.detected = detected
        obs.image_width, obs.image_height = 640, 480
        obs.center_x, obs.center_y = 320, 240
        obs.pixel_height = 640/(2*math.tan(math.radians(44)))*.297/math.hypot(marker_x-.145, marker_y)
        obs.pixel_width = .7*obs.pixel_height
        obs.contour_ratio = area
        obs_pub.publish(obs)
        nav = Twist(); nav.linear.x = .071; nav.angular.z = .11
        if nav_state is not None:
            nav_state_pub.publish(nav_state)
            nav.linear.x, nav.angular.z = .50, .60
        visual = Twist(); visual.linear.x = .039; visual.angular.z = -.13
        nav_pub.publish(nav)
        visual_pub.publish(visual)
        if goal:
            visual_goal_pub.publish(True)
        rate.sleep()
    return start, time.monotonic()

try:
    if oblique_case:
        angle = math.radians(35)
        moving = run_phase(True, wall_angle=angle, nav_state='APPROACH: navigating')
        stationary_state = ('APPROACH: entrance reached' if oblique_case == 'oblique-entrance'
                            else 'APPROACH: waiting for a clear blue-marker entrance')
        if oblique_case == 'oblique-stalled':
            wait = run_phase(True, wall_angle=angle, nav_state=stationary_state, duration=1.0)
            assert not any(enabled for _, enabled in visual_enable), 'Switched before stall delay'
        blocked = run_phase(True, wall_angle=angle, nav_state=stationary_state,
                            obstacle=True, duration=2.0)
        clear = run_phase(True, wall_angle=angle, nav_state=stationary_state, duration=1.0)
    else:
        missing = run_phase(False)
        far = run_phase(True, wall_x=2.0)
        mismatch = run_phase(True, foreground_wall=True, duration=1.5)
        blocked = run_phase(True, obstacle=True, duration=1.6)
        clear = run_phase(True, duration=2.0)
        visual = run_phase(True, duration=.9)
        finish = run_phase(True, goal=True, duration=.9)
finally:
    proc.terminate()
    try:
        proc.wait(timeout=4)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()

def output(interval):
    return [(v,w) for t,v,w in commands if interval[0]+.3 < t < interval[1]]

if oblique_case:
    assert not any(enabled and t < clear[0] for t, enabled in visual_enable), 'Switched with moving base or obstacle'
    assert any(enabled and clear[0] <= t < clear[1] for t, enabled in visual_enable), '35° clear corridor did not hand off'
    assert any(s.startswith('TERMINAL_PARK:') for _, s in states)
    assert any(abs(v-.50*2/3)<1e-6 and abs(w-.40)<1e-6 for v,w in output(moving)), 'APPROACH arc limit changed curvature'
    print(f'PASS: {oblique_case}, 35° wall, stopped/blocked gates and APPROACH command curvature')
    rospy.signal_shutdown('oblique handoff mock complete')
    sys.exit(0)

print('states:', [s for _,s in states])
print('nav_enable:', nav_enable)
print('visual_enable:', visual_enable)
print('goal:', goals)
print('latest commands:', {n: output(i)[-3:] for n,i in
                           [('missing',missing),('mismatch',mismatch),('blocked',blocked),('clear',clear),('visual',visual),('finish',finish)]})
assert any(abs(v-.071)<1e-6 for v,w in output(missing)), 'Navigation commands not forwarded initially'
assert any(abs(v-.071)<1e-6 for v,w in output(far)), 'Long range did not retain navigation'
assert any(abs(v-.071)<1e-6 for v,w in output(mismatch)), 'Foreground wall took control'
assert any(abs(v-.071)<1e-6 for v,w in output(blocked)), 'Blocked corridor did not keep navigation'
assert not any(v and t < clear[0] for t,v in visual_enable), 'Switched before the corridor cleared'
assert any(v and clear[0] <= t < clear[1] for t,v in visual_enable), 'Clear corridor did not trigger visual mode'
assert any(abs(v-.039)<1e-6 for v,w in output(visual)), 'Visual commands not forwarded after switch'
assert not any(abs(v-.071)<1e-6 for v,w in output(visual)), 'Navigation still controlled the base after switch'
assert any(v and t >= finish[0] for t,v in goals), 'Visual goal did not latch final stop'
assert not any(abs(v)>1e-9 or abs(w)>1e-9 for v,w in output(finish)), 'Commands leaked after completion'
print('PASS: target plus clear corridor switches command ownership and latches visual stop')
rospy.signal_shutdown('visual handoff offline mock complete')
