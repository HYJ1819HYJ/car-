#!/usr/bin/env python3
"""One-shot simulated start-to-finish run with ground-truth collision audit."""
import json
import math
from pathlib import Path
import threading
import time

import actionlib
import cv2
import numpy as np
import rospkg
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import LaserScan
from PIL import Image
from std_msgs.msg import String
import yaml


def main():
    rospy.init_node('competition_demo')
    if rospy.get_param('/competition_simulation', False) is not True:
        raise RuntimeError('This demo requires the isolated competition simulation.')
    pkg = Path(rospkg.RosPack().get_path('competition_sim'))
    geometry = json.loads((pkg / 'maps/geometry.json').read_text())
    scenario = json.loads(Path(rospy.get_param('~scenario_file')).read_text())
    footprint = np.asarray(rospy.get_param('/robot_geometry/footprint'), dtype=float)
    image = np.asarray(Image.open(pkg / 'maps/final_competition.pgm'))
    saved_image = np.asarray(Image.open(pkg / 'maps/mapping_before_move.pgm'))
    config = yaml.safe_load((pkg / 'maps/final_competition.yaml').read_text())
    movable_mask=np.asarray(Image.open(pkg / 'maps/movable_prior_mask.png'))
    saved_occupancy=np.flipud(np.where(saved_image==0,100,np.where(saved_image==254,0,-1)))
    mutable=np.flipud(movable_mask)>0
    origin = np.asarray(config['origin'][:2])
    resolution = config['resolution']
    lock = threading.RLock()
    report = {'state': 'WAITING', 'ground_truth_samples': 0, 'collision_samples': 0,
              'min_clearance_m': None, 'reverse_commands': 0,
              'max_linear_command': 0.0, 'max_angular_command': 0.0,
              'global_plan_messages': 0, 'local_plan_messages': 0,
              'distance_m': 0.0, 'trace': [], 'goal': geometry['finish']}
    report.update({'scenario':scenario['id'],'expected':scenario['expected'],
                   'base_navigation_map_matches':False,
                   'semantics':scenario['semantics'],'mapped_boxes':scenario['mapped_boxes'],
                   'physical_boxes':scenario['physical_boxes'],'map_reconciliation':None,
                   'active_fixed_and_unknown_unchanged':False,'active_prior_cleared_cells':0,
                   'prior_costmap_nonlethal_cells_max':0,
                   'added_collision_samples':0,'min_added_clearance_m':None,
                   'added_boxes':scenario['boxes'],
                   'laser_frames_seen':{str(b['id']):0 for b in scenario['boxes']},
                   'costmap_frames_seen':{str(b['id']):0 for b in scenario['boxes']}})
    last_scan_check = [0.0]
    def map_check(msg):
        expected = np.flipud(np.where(saved_image==0,100,np.where(saved_image==254,0,-1))).ravel()
        matches = (msg.info.width==image.shape[1] and msg.info.height==image.shape[0]
                   and abs(msg.info.resolution-resolution)<1e-8
                   and np.array_equal(np.asarray(msg.data),expected))
        with lock:
            report['base_navigation_map_matches']=bool(matches)
    def active_map_check(msg):
        actual=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
        with lock:
            report['active_fixed_and_unknown_unchanged']=bool(actual.shape==saved_occupancy.shape and
                np.array_equal(actual[~mutable],saved_occupancy[~mutable]))
            report['active_prior_cleared_cells']=int(np.sum(actual[mutable]==0))
    def scan_check(msg):
        if report['state'] in ('SUCCEEDED','FAILED'):
            return
        t = msg.header.stamp.to_sec()
        with lock:
            if not report['trace'] or t-last_scan_check[0]<.18:
                return
            pose = min(report['trace'][-10:],key=lambda p:abs(p[0]-t))
        if abs(pose[0]-t)>.05:
            return
        last_scan_check[0]=t
        _,x,y,yaw = pose
        ranges=np.asarray(msg.ranges)
        angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment+yaw
        px=x+.315*math.cos(yaw)+ranges*np.cos(angles)
        py=y+.315*math.sin(yaw)+ranges*np.sin(angles)
        with lock:
            for box in scenario['boxes']:
                dx,dy=np.abs(px-box['center'][0]),np.abs(py-box['center'][1])
                hx,hy=np.asarray(box['size'][:2])/2
                seen=(dx<hx+.04)&(dy<hy+.04)&((np.abs(dx-hx)<.04)|(np.abs(dy-hy)<.04))
                report['laser_frames_seen'][str(box['id'])]+=int(np.any(seen))
    def costmap_check(msg):
        if report['state'] in ('SUCCEEDED','FAILED'):
            return
        grid=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
        with lock:
            if (grid.shape==mutable.shape and abs(msg.info.resolution-resolution)<1e-8 and
                abs(msg.info.origin.position.x-config['origin'][0])<1e-6 and abs(msg.info.origin.position.y-config['origin'][1])<1e-6):
                report['prior_costmap_nonlethal_cells_max']=max(report['prior_costmap_nonlethal_cells_max'],
                    int(np.sum((grid[mutable]>=0)&(grid[mutable]<99))))
            for box in scenario['boxes']:
                lo=np.asarray(box['center'])-np.asarray(box['size'][:2])/2-.08
                hi=np.asarray(box['center'])+np.asarray(box['size'][:2])/2+.08
                origin=np.array([msg.info.origin.position.x,msg.info.origin.position.y])
                a=np.floor((lo-origin)/msg.info.resolution).astype(int)
                b=np.ceil((hi-origin)/msg.info.resolution).astype(int)
                a=np.maximum(a,[0,0]);b=np.minimum(b,[msg.info.width,msg.info.height])
                if np.all(b>a):
                    report['costmap_frames_seen'][str(box['id'])]+=int(np.any(grid[a[1]:b[1],a[0]:b[0]]>=99))
    trajectory_pub = rospy.Publisher('/simulation/ground_truth_path', NavPath, queue_size=1, latch=True)
    trajectory = NavPath()
    trajectory.header.frame_id = 'map'
    distances = cv2.distanceTransform((image == 254).astype('uint8'), cv2.DIST_L2, 5)*resolution
    def ground_truth(msg):
        if report['state'] in ('SUCCEEDED','FAILED'):
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = 2*math.atan2(q.z, q.w)
        c,s = math.cos(yaw), math.sin(yaw)
        polygon = footprint @ np.asarray([[c,s],[-s,c]]) + [p.x,p.y]
        pixels = np.floor((polygon-origin)/resolution).astype('int32')
        pixels[:,1] = image.shape[0]-1-pixels[:,1]
        mask = np.zeros(image.shape, dtype='uint8')
        cv2.fillPoly(mask, [pixels], 1)
        outside = (pixels[:,0].min()<0 or pixels[:,0].max()>=image.shape[1] or
                   pixels[:,1].min()<0 or pixels[:,1].max()>=image.shape[0])
        collision = outside or bool(np.any(image[mask != 0] != 254))
        clearance = float(distances[mask != 0].min()) if np.any(mask) else 0.0
        added_collision=False
        added_clearance=float('inf')
        for box in scenario['boxes']:
            square=np.asarray(box['corners'],np.float32)
            area,_=cv2.intersectConvexConvex(polygon.astype(np.float32),square)
            added_collision |= area>1e-8
            for points,edges in [(polygon,square),(square,polygon)]:
                delta=np.roll(edges,-1,axis=0)-edges
                difference=points[:,None,:]-edges[None,:,:]
                fraction=np.clip(np.sum(difference*delta,axis=2)/np.sum(delta*delta,axis=1),0,1)
                gap=float(np.linalg.norm(difference-fraction[:,:,None]*delta[None,:,:],axis=2).min())
                added_clearance=min(added_clearance,gap)
        with lock:
            trace = report['trace']
            if trace:
                report['distance_m'] += math.hypot(p.x-trace[-1][1],p.y-trace[-1][2])
            trace.append([msg.header.stamp.to_sec(), p.x,p.y,yaw])
            report['ground_truth_samples'] += 1
            velocity=msg.twist.twist
            report['final_actual_velocity']=[math.hypot(velocity.linear.x,velocity.linear.y),velocity.angular.z]
            report['collision_samples'] += int(collision)
            report['added_collision_samples'] += int(added_collision)
            previous_added=report['min_added_clearance_m']
            report['min_added_clearance_m']=added_clearance if previous_added is None else min(previous_added,added_clearance)
            previous = report['min_clearance_m']
            report['min_clearance_m'] = clearance if previous is None else min(previous,clearance)
            # The trace uses Stage ground truth, independently of localization.
            if len(trace) % 3 == 0:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = msg.header.stamp
                pose.pose = msg.pose.pose
                trajectory.poses.append(pose)
                trajectory.header.stamp = msg.header.stamp
                trajectory_pub.publish(trajectory)
    def command(msg):
        with lock:
            if report['state'] in ('SUCCEEDED','FAILED'):
                return
            report['reverse_commands'] += int(msg.linear.x < -1e-6)
            report['max_linear_command'] = max(report['max_linear_command'],abs(msg.linear.x))
            report['max_angular_command'] = max(report['max_angular_command'],abs(msg.angular.z))
    def plan(msg,key):
        if msg.poses:
            with lock:
                if report['state'] in ('SUCCEEDED','FAILED'):
                    return
                report[key] += 1
    rospy.Subscriber('/base_pose_ground_truth', Odometry, ground_truth, queue_size=10)
    rospy.Subscriber('/simulation/cmd_vel', Twist, command, queue_size=10)
    def reconciliation(msg):
        with lock: report['map_reconciliation']=json.loads(msg.data)
    rospy.Subscriber('/simulation/map_reconciliation',String,reconciliation,queue_size=1)
    rospy.Subscriber('/map', OccupancyGrid, map_check, queue_size=1)
    rospy.Subscriber('/navigation_map', OccupancyGrid, active_map_check, queue_size=1)
    rospy.Subscriber('/scan_raw', LaserScan, scan_check, queue_size=1)
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, costmap_check, queue_size=1)
    rospy.Subscriber('/move_base/GlobalPlanner/plan', NavPath,
                     lambda m: plan(m,'global_plan_messages'), queue_size=1)
    rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', NavPath,
                     lambda m: plan(m,'local_plan_messages'), queue_size=1)
    status = rospy.Publisher('/simulation/demo_status', String, queue_size=1, latch=True)
    client = actionlib.SimpleActionClient('/move_base',MoveBaseAction)
    client.wait_for_server()
    rospy.sleep(5)
    x,y,yaw = geometry['finish']
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = x,y
    goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
    report['state'] = 'RUNNING'
    status.publish(report['state'])
    client.send_goal(goal)
    rospy.loginfo('SIMULATION ONLY: navigating from yellow start to example finish.')
    deadline = time.monotonic()+150
    while not rospy.is_shutdown() and time.monotonic()<deadline:
        if client.wait_for_result(rospy.Duration(0.5)):
            break
    state = client.get_state()
    if state not in (2,3,4,5,8,9):
        client.cancel_goal()
    rospy.sleep(1.0)  # Observe the virtual chassis braking after terminal state.
    with lock:
        report['state'] = 'SUCCEEDED' if state == 3 else 'FAILED'
        report['action_state'] = state
        report['action_text'] = client.get_goal_status_text()
        report['stationary_final']=bool(len(report['trace'])>30 and
            math.hypot(report['trace'][-1][1]-report['trace'][-30][1],
                       report['trace'][-1][2]-report['trace'][-30][2])<.01)
        if report['trace']:
            p = report['trace'][-1]
            report['goal_position_error_m'] = math.hypot(p[1]-x,p[2]-y)
            report['goal_yaw_error_rad'] = abs(math.atan2(math.sin(p[3]-yaw),math.cos(p[3]-yaw)))
        output = Path(rospy.get_param('~report_file','/tmp/competition_sim_result.json'))
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(report,indent=2)+'\n')
        status.publish(report['state'])
        rospy.loginfo('Simulation result: %s; %s', report['state'],output)
    rospy.spin()


if __name__ == '__main__':
    main()
