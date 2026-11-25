#!/usr/bin/env python3
"""Run navigation, handoff and visual nodes with simulated ROS messages only."""
import math
import os
import subprocess
import time

import actionlib
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from stm32_bridge.msg import BlueTargetObservation

if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11329':
    raise SystemExit('This mock only runs on isolated ROS master port 11329')
rospy.init_node('handoff_pipeline_mock')
hold_test = os.environ.get('COURSE_TEST_FINAL_HOLD') == '1'
rospy.set_param('/robot_geometry/footprint', [[.3483,.245],[.3483,-.245],
                                              [-.1517,-.245],[-.1517,.245]])
for key, val in {'auto_start':False,'start_delay':0.0,'require_localization':False,
                 'require_gimbal':True,'target_sample_count':3,'line_guidance_enabled':False,
                 'acquire_target_in_navigation':False,
                 'hold_at_course_end':hold_test,
                 'course_waypoints':[[.8, 0.0], [1.5, -.4]],
                 'approach_clearance':.37}.items():
    rospy.set_param('/handoff_blue_navigator/'+key,val)
rospy.set_param('/handoff_direct_visual/auto_start',False)
rospy.set_param('/handoff_direct_visual/require_gimbal',True)
rospy.set_param('/visual_handoff_manager/auto_start',True)
rospy.set_param('/visual_handoff_manager/required_clear_scans',3)
goals, preempted, commands, states, nav_states, reached = [], [], [], [], [], []

def execute(goal):
    goals.append((time.monotonic(),goal))
    first_goal = len(goals) == 1
    started = time.monotonic()
    while not rospy.is_shutdown():
        if server.is_preempt_requested():
            preempted.append(time.monotonic())
            server.set_preempted()
            return
        if (first_goal or hold_test) and time.monotonic() - started > .4:
            server.set_succeeded()
            return
        rospy.sleep(.04)

server=actionlib.SimpleActionServer('/move_base',MoveBaseAction,execute_cb=execute,auto_start=False)
server.start()
obs_pub=rospy.Publisher('/blue_target/observation',BlueTargetObservation,queue_size=2)
scan_pub=rospy.Publisher('/scan',LaserScan,queue_size=2)
joint_pub=rospy.Publisher('/gimbal/joint_states',JointState,queue_size=2)
connected=rospy.Publisher('/gimbal/connected',Bool,queue_size=1,latch=True)
ready=rospy.Publisher('/gimbal/servo_ready',Bool,queue_size=1,latch=True)
locked=rospy.Publisher('/gimbal/target_locked',Bool,queue_size=1,latch=True)
map_pub=rospy.Publisher('/map',OccupancyGrid,queue_size=1,latch=True)
fake_nav=rospy.Publisher('/handoff/cmd_vel_nav',Twist,queue_size=2)
rospy.Subscriber('/cmd_vel',Twist,lambda m: commands.append((time.monotonic(),m.linear.x,m.angular.z)),queue_size=40)
rospy.Subscriber('/blue_autonomy/state',String,lambda m: states.append((time.monotonic(),m.data)),queue_size=20)
rospy.Subscriber('/handoff/nav_state',String,lambda m: nav_states.append(m.data),queue_size=20)
rospy.Subscriber('/goal_reached',Bool,lambda m: reached.append((time.monotonic(),m.data)),queue_size=10)
connected.publish(True); ready.publish(True); locked.publish(True)
grid=OccupancyGrid(); grid.header.frame_id='map'; grid.info.resolution=.025
grid.info.width=160; grid.info.height=120; grid.info.origin.position.x=-1.0
grid.info.origin.position.y=-1.5; grid.info.origin.orientation.w=1
grid.data=[100 if -1+(i%160+.5)*.025 >= 2 else 0 for i in range(grid.info.width*grid.info.height)]
map_pub.publish(grid)
tf_pub=tf2_ros.TransformBroadcaster()
static=tf2_ros.StaticTransformBroadcaster()
tf_laser=TransformStamped(); tf_laser.header.stamp=rospy.Time.now()
tf_laser.header.frame_id='base_footprint'; tf_laser.child_frame_id='laser'
tf_laser.transform.translation.x=.315; tf_laser.transform.rotation.w=1
static.sendTransform(tf_laser)
commands_to_run=[
 ['rosrun','maze_planner','blue_goal_navigator','__name:=handoff_blue_navigator',
  '/blue_autonomy/enable:=/handoff/nav_enable','/blue_autonomy/state:=/handoff/nav_state',
  '/goal_reached:=/handoff/nav_goal_reached'],
 ['rosrun','maze_planner','direct_visual_controller','__name:=handoff_direct_visual',
  '/blue_autonomy/enable:=/handoff/visual_enable','/blue_autonomy/state:=/handoff/visual_state',
  '/goal_reached:=/handoff/visual_goal_reached','/cmd_vel:=/handoff/cmd_vel_visual'],
 ['rosrun','maze_planner','visual_handoff_manager','__name:=visual_handoff_manager']]
children=[subprocess.Popen(cmd) for cmd in commands_to_run]

def phase(detected, obstacle=False, area=.02, duration=1.8):
    start=time.monotonic(); rate=rospy.Rate(20)
    while time.monotonic()-start < duration and not rospy.is_shutdown():
        stamp=rospy.Time.now()
        tf=TransformStamped(); tf.header.stamp=stamp; tf.header.frame_id='map'
        tf.child_frame_id='base_footprint'; tf.transform.rotation.w=1; tf_pub.sendTransform(tf)
        scan=LaserScan(); scan.header.stamp=stamp; scan.header.frame_id='laser'
        scan.angle_min=-math.pi; scan.angle_increment=2*math.pi/1440
        scan.angle_max=math.pi-scan.angle_increment; scan.range_min=.1; scan.range_max=10
        scan.ranges=[6.0]*1440
        for i in range(1440):
            a=scan.angle_min+i*scan.angle_increment
            if math.cos(a)>.2: scan.ranges[i]=(2-.315)/math.cos(a)
        if obstacle:
            a=math.atan2(.15,1-.315); beam=round((a-scan.angle_min)/scan.angle_increment)
            for i in range(beam-4,beam+5): scan.ranges[i]=math.hypot(1-.315,.15)
        scan_pub.publish(scan)
        joint=JointState(); joint.header.stamp=stamp
        joint.name=['camera_pan_joint','camera_tilt_joint']; joint.position=[0.,0.]
        joint_pub.publish(joint)
        obs=BlueTargetObservation(); obs.header.stamp=stamp; obs.detected=detected
        obs.image_width,obs.image_height=640,480; obs.center_x,obs.center_y=320,240
        obs.pixel_height=640/(2*math.tan(math.radians(35)))*.297/(2-.145)
        obs.pixel_width=.7*obs.pixel_height; obs.contour_ratio=area; obs_pub.publish(obs)
        nav=Twist(); nav.linear.x=.071; nav.angular.z=.11; fake_nav.publish(nav)
        rate.sleep()
    return start,time.monotonic()

try:
    if hold_test:
        phase(False,duration=7.0)
    else:
        search=phase(False,duration=2.3)
        blocked=phase(True,obstacle=True,duration=2.1)
        clear=phase(True,duration=2.4)
        finish=phase(True,area=.16,duration=1.0)
finally:
    for child in children: child.terminate()
    for child in children:
        try: child.wait(timeout=4)
        except subprocess.TimeoutExpired: child.kill(); child.wait()

print('states:',[s for _,s in states])
print('navigation goals:',len(goals),'cancellations:',len(preempted))
print('goal reached:',reached)
def interval_values(interval): return [(v,w) for t,v,w in commands if interval[0]+.4<t<interval[1]]
assert goals, 'Navigator never began an autonomous search goal'
assert len(goals) >= 2, 'Navigator did not advance along the mapped course'
for (_, goal), expected in zip(goals[:2], ((.8, 0.0), (1.5, -.4))):
    p = goal.target_pose.pose.position
    assert abs(p.x - expected[0]) < 1e-6 and abs(p.y - expected[1]) < 1e-6, \
        'Mapped course was not followed in order'
if hold_test:
    assert len(goals) == 2, 'Navigator sent an unrelated goal after the mapped course'
    assert any('mapped course end reached' in state for state in nav_states), \
        'Navigator did not wait at the finish area while scanning'
    print('PASS: final mapped waypoint holds position for visual search')
    rospy.signal_shutdown('course end mock complete')
    raise SystemExit(0)
assert not any(t < clear[0] for t in preempted), \
    'Navigation was canceled by the camera while the corridor was blocked'
assert preempted, 'Navigation goal was not canceled during handoff/search transition'
assert not any(s.startswith('VISUAL:') and t<clear[0] for t,s in states), 'Switched with obstructed corridor'
assert any(s.startswith('VISUAL:') and clear[0]<=t<clear[1] for t,s in states), 'No visual handoff after corridor cleared'
assert any(abs(v-.1)<1e-6 for v,w in interval_values(clear)), 'Direct visual command was not forwarded'
assert not any(abs(v-.071)<1e-6 for v,w in interval_values(finish)), 'Navigation command leaked after handoff'
assert any(v and t>=finish[0] for t,v in reached), 'Visual stop did not latch the global goal'
print('PASS: mapped course, obstacle-gated handoff, direct control, and final latch')
rospy.signal_shutdown('handoff pipeline mock complete')
