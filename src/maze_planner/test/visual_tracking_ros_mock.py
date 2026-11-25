#!/usr/bin/env python3
"""Exercise visual following on isolated ROS master 11329; no hardware nodes."""
import math
import os
import subprocess
import threading
import time

import actionlib
import rospy
import tf2_ros
from dynamic_reconfigure.srv import Reconfigure, ReconfigureResponse
from geometry_msgs.msg import TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from stm32_bridge.msg import BlueTargetObservation

if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11329':
    raise SystemExit('This mock only runs on the isolated ROS master at port 11329')

rospy.init_node('visual_tracking_offline_mock')
start_unseen = os.environ.get('VISUAL_MOCK_START_UNSEEN') == '1'
early_success = os.environ.get('VISUAL_MOCK_EARLY_SUCCESS') == '1'
mutex = threading.Lock()
pose = [0.0, 0.0, 0.0]
target = (2.0, -.5)
goals = []
approach_goals = []
states = []
done = []
second_goal_at = [None]
search_goal_at = [None]
preempted = []
reconfigurations = []

def reconfigure(request):
    values = {item.name: item.value for item in request.config.doubles}
    reconfigurations.append(values)
    for name, value in values.items():
        rospy.set_param('/move_base/TebLocalPlannerROS/' + name, value)
    return ReconfigureResponse(config=request.config)

rospy.set_param('/move_base/TebLocalPlannerROS/xy_goal_tolerance', .12)
rospy.set_param('/move_base/TebLocalPlannerROS/yaw_goal_tolerance', .12)
reconfigure_service = rospy.Service('/move_base/TebLocalPlannerROS/set_parameters',
                                    Reconfigure, reconfigure)

def execute(goal):
    p = goal.target_pose.pose.position
    q = goal.target_pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
    with mutex:
        goals.append((p.x, p.y, yaw))
        count = len(goals)
        # An initial search can win the startup race against the first
        # camera observation. The fixture's search point is far from the marker.
        if count == 1 and (start_unseen or math.hypot(p.x - target[0], p.y - target[1]) > 1.2):
            search_goal_at[0] = time.monotonic()
        else:
            approach_goals.append((p.x, p.y, yaw))
        approach_count = len(approach_goals)
        if approach_count == 1:
            if early_success:
                # Simulate move_base reporting success before the visual
                # marker is large enough to satisfy the stopping condition.
                pose[:] = (p.x, p.y, yaw)
            else:
                # A detour changes the target bearing while en route.
                pose[:] = (.35, .25, 0.0)
        elif approach_count >= 2:
            pose[:] = (p.x, p.y, yaw)
            if second_goal_at[0] is None:
                second_goal_at[0] = time.monotonic()
    for iteration in range(160):
        if server.is_preempt_requested():
            preempted.append(count)
            server.set_preempted()
            return
        if early_success and approach_count == 1 and iteration == 6:
            server.set_succeeded(MoveBaseResult())
            return
        rospy.sleep(.05)
    server.set_aborted()

server = actionlib.SimpleActionServer('/move_base', MoveBaseAction, execute_cb=execute,
                                      auto_start=False)
server.start()
map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
obs_pub = rospy.Publisher('/blue_target/observation', BlueTargetObservation, queue_size=1)
joint_pub = rospy.Publisher('/gimbal/joint_states', JointState, queue_size=1)
connected_pub = rospy.Publisher('/gimbal/connected', Bool, queue_size=1, latch=True)
ready_pub = rospy.Publisher('/gimbal/servo_ready', Bool, queue_size=1, latch=True)
locked_pub = rospy.Publisher('/gimbal/target_locked', Bool, queue_size=1, latch=True)
state_sub = rospy.Subscriber('/blue_autonomy/state', String,
                             lambda m: states.append(m.data), queue_size=10)
done_sub = rospy.Subscriber('/goal_reached', Bool,
                            lambda m: done.append((time.monotonic(), m.data)), queue_size=10)
broadcaster = tf2_ros.TransformBroadcaster()
static = tf2_ros.StaticTransformBroadcaster()
laser = TransformStamped()
laser.header.stamp = rospy.Time.now()
laser.header.frame_id = 'base_footprint'
laser.child_frame_id = 'laser'
laser.transform.translation.x = .315
laser.transform.translation.z = .33
laser.transform.rotation.w = 1.0
static.sendTransform(laser)
connected_pub.publish(True)
ready_pub.publish(True)
locked_pub.publish(False)  # Visual mode should still use fresh pan feedback.

grid = OccupancyGrid()
grid.header.frame_id = 'map'
grid.info.resolution = .025
grid.info.width = 160
grid.info.height = 120
grid.info.origin.position.x = -1.0
grid.info.origin.position.y = -1.5
grid.info.origin.orientation.w = 1.0
grid.data = [100 if (-1.0 + (i % 160 + .5)*.025) >= target[0] else 0
             for i in range(grid.info.width * grid.info.height)]
map_pub.publish(grid)
rospy.set_param('/robot_geometry/footprint', [[.3483,.245],[.3483,-.245],[-.1517,-.245],[-.1517,.245]])
params = {'auto_start': True, 'start_delay': 0.0, 'require_gimbal': True,
          'require_localization': False, 'line_guidance_enabled': False,
          'wall_parking_enabled': False, 'visual_tracking_parking_enabled': True,
          'visual_stop_area_ratio': .16, 'visual_facing_tolerance_deg': 5.0,
          'visual_approach_standoff': .48, 'visual_approach_step': .05,
          'visual_goal_update_period': .5,
          'approach_clearance': .37, 'target_sample_count': 3}
for name, value in params.items():
    rospy.set_param('/blue_goal_navigator/' + name, value)

proc = subprocess.Popen(['rosrun', 'maze_planner', 'blue_goal_navigator',
                         '__name:=blue_goal_navigator'])
fx = 640 / (2*math.tan(math.radians(35)))
start = time.monotonic()
rate = rospy.Rate(20)
try:
    while not rospy.is_shutdown() and time.monotonic()-start < 20 and not any(v for _,v in done):
        with mutex:
            x, y, yaw = pose
            approach_count = len(approach_goals)
            goal_time = second_goal_at[0]
            search_time = search_goal_at[0]
        now = rospy.Time.now()
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(yaw/2)
        tf.transform.rotation.w = math.cos(yaw/2)
        broadcaster.sendTransform(tf)

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'laser'
        scan.angle_min = -1.4
        scan.angle_increment = .005
        scan.angle_max = 1.4
        scan.range_min = .05
        scan.range_max = 5.0
        lx = x+.315*math.cos(yaw)
        ly = y+.315*math.sin(yaw)
        for i in range(561):
            a = yaw+scan.angle_min+i*scan.angle_increment
            r = (target[0]-lx)/math.cos(a) if math.cos(a) > 0 else float('inf')
            iy = ly+r*math.sin(a) if math.isfinite(r) else float('inf')
            scan.ranges.append(r if .05 < r < 5 and abs(iy) < 2 else float('inf'))
        scan_pub.publish(scan)

        dx = target[0] - (x+.145*math.cos(yaw))
        dy = target[1] - (y+.145*math.sin(yaw))
        pan = math.atan2(dy, dx) - yaw
        # After the second goal, area is large enough, but the camera initially
        # reports a 0.2-rad bearing error; stopping must wait for alignment.
        if approach_count >= 2 and goal_time is not None and time.monotonic()-goal_time < .4:
            pan += .2
        joints = JointState()
        joints.header.stamp = now
        joints.name = ['camera_pan_joint', 'camera_tilt_joint']
        joints.position = [pan, 0.0]
        joint_pub.publish(joints)

        obs = BlueTargetObservation()
        obs.header.stamp = now
        # In the unseen case, publish valid empty frames until the first
        # search waypoint is active, then reveal the marker to the detector.
        obs.detected = not start_unseen or (search_time is not None and
                                           time.monotonic()-search_time >= .3)
        obs.image_width = 640
        obs.image_height = 480
        obs.center_x = 320
        obs.center_y = 240
        obs.pixel_height = fx*.297/max(.25, math.hypot(dx, dy))
        obs.pixel_width = obs.pixel_height*.7
        obs.contour_ratio = .20 if approach_count >= 2 else .04
        obs_pub.publish(obs)
        rate.sleep()
finally:
    proc.terminate()
    try:
        proc.wait(timeout=4)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()

print('states:', states)
print('goals:', goals)
print('preempted:', preempted)
print('goal_reached:', done)
assert any(value for _,value in done), 'Visual area/facing condition did not stop the mock'
assert len(approach_goals) >= 2, 'The target direction was not updated after the detour'
if early_success:
    first_range = math.hypot(approach_goals[0][0]-target[0], approach_goals[0][1]-target[1])
    second_range = math.hypot(approach_goals[1][0]-target[0], approach_goals[1][1]-target[1])
    assert first_range - second_range > .04, 'Unmet visual threshold did not request a closer goal'
    assert any('planning closer' in s for s in states), 'Early goal success incorrectly ended the mission'
else:
    assert approach_goals[0][2] < 0 and approach_goals[1][2] < approach_goals[0][2]-.08, 'Goal yaw did not follow target bearing'
reached_at = next(stamp for stamp,value in done if value)
assert second_goal_at[0] is not None and reached_at-second_goal_at[0] >= .35, 'Stopped before facing the target'
if not early_success:
    assert 1 + int(search_goal_at[0] is not None) in preempted, 'The first approach goal was not updated after the bearing changed'
assert any(abs(c.get('xy_goal_tolerance', 0)-.04) < 1e-6 for c in reconfigurations), 'Visual goal tolerance was not applied'
assert any(abs(c.get('xy_goal_tolerance', 0)-.12) < 1e-6 for c in reconfigurations), 'Original goal tolerance was not restored'
if start_unseen:
    assert any(s.startswith('SEARCH:') for s in states), 'Unseen target did not start map search'
    assert 1 in preempted, 'The search goal was not canceled when the target appeared'
print('PASS: fresh gimbal bearing updates goals; 16% AND 5-degree condition ends the mock')
if start_unseen:
    print('PASS: initially unseen target triggers search, then cancels search and is followed')
if early_success:
    print('PASS: an early move_base success requests a closer goal until visual stop conditions are met')
rospy.signal_shutdown('visual tracking offline mock complete')
