#!/usr/bin/env python3
"""Clear only known movable prior cells after repeated, unobstructed laser rays.

Inputs contain the saved BEFORE map and its movable-cell labels, never the
AFTER scene or Stage ground truth. Fixed and unknown cells remain unchanged.
"""
import json
import math
from pathlib import Path
import threading
import numpy as np
from PIL import Image
import yaml


class FreeEvidence:
    def __init__(self, labels, required=3):
        self.labels=np.asarray(labels)
        self.rows,self.cols=np.nonzero(self.labels)
        self.ids=self.labels[self.rows,self.cols]
        self.counts=np.zeros(len(self.rows),np.uint8)
        self.cleared=np.zeros(len(self.rows),bool)
        self.required=required
        self.last_stamp=-1.
        self.frames=0

    def observe(self, stamp, free):
        if stamp<=self.last_stamp:return False
        self.last_stamp=stamp;self.frames+=1
        self.counts=np.where(free,np.minimum(self.counts+1,self.required),0).astype(np.uint8)
        previous=self.cleared.copy()
        self.cleared|=self.counts>=self.required
        return bool(np.any(previous!=self.cleared))


def ray_free(points, laser_pose, ranges, angle_min, angle_increment,
             range_min, range_max, limit=5., margin=.08):
    x,y,yaw=laser_pose
    delta=points-[x,y]
    distance=np.linalg.norm(delta,axis=1)
    angle=np.arctan2(delta[:,1],delta[:,0])-yaw
    angle=np.arctan2(np.sin(angle),np.cos(angle))
    index=np.rint((angle-angle_min)/angle_increment).astype(int)
    # Adjacent beams must also pass beyond the candidate cell. NaNs represent
    # self occlusion; infinity is valid free range only up to the raytrace limit.
    samples=np.asarray(ranges)[np.clip(index[:,None]+[-1,0,1],0,len(ranges)-1)]
    valid=np.all((samples>=range_min)&~np.isnan(samples),axis=1)
    observed=np.minimum(np.min(samples,axis=1),min(limit,range_max))
    return valid&(index>0)&(index<len(ranges)-1)&(distance>=range_min)&(observed>distance+margin)


def main():
    import rospy
    import tf2_ros
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    rospy.init_node('reconcile_prior_map')
    if rospy.get_param('/competition_simulation',False) is not True:
        raise RuntimeError('This configured movable-prior adapter is simulation-only')
    config_path=Path(rospy.get_param('~map_file'))
    config=yaml.safe_load(config_path.read_text())
    image=np.flipud(np.asarray(Image.open(config_path.parent/config['image'])))
    labels=np.flipud(np.asarray(Image.open(rospy.get_param('~movable_mask'))))
    original=np.where(image==0,100,np.where(image==254,0,-1)).astype(np.int8)
    if labels.shape!=original.shape or np.any(original[labels>0]!=100):
        raise ValueError('Movable mask must select occupied cells of the saved map')
    evidence=FreeEvidence(labels)
    grid=original.copy();lock=threading.RLock()
    res=config['resolution'];ox,oy,_=config['origin']
    points=np.column_stack((ox+(evidence.cols+.5)*res,oy+(evidence.rows+.5)*res))
    buffer=tf2_ros.Buffer(rospy.Duration(10));listener=tf2_ros.TransformListener(buffer)
    publisher=rospy.Publisher('/navigation_map',OccupancyGrid,queue_size=1,latch=True)
    diagnostic=rospy.Publisher('/simulation/map_reconciliation',String,queue_size=1,latch=True)
    failures=[0];last_publish=[-1.]
    def publish():
        with lock:
            message=OccupancyGrid();message.header.frame_id='map';message.header.stamp=rospy.Time.now()
            message.info.resolution=res;message.info.width=image.shape[1];message.info.height=image.shape[0]
            message.info.origin.position.x=ox;message.info.origin.position.y=oy;message.info.origin.orientation.w=1.
            message.data=grid.ravel().tolist();publisher.publish(message)
            details={'input':'saved_before_map_and_laser_only','scan_frames':evidence.frames,'tf_failures':failures[0],
                     'required_free_observations':evidence.required,
                     'fixed_and_unknown_unchanged':bool(np.array_equal(grid[labels==0],original[labels==0])),
                     'boxes':{str(i):{'prior_cells':int(np.sum(evidence.ids==i)),
                                      'cleared_cells':int(np.sum(evidence.cleared&(evidence.ids==i)))}
                              for i in np.unique(evidence.ids)}}
            diagnostic.publish(String(data=json.dumps(details)))
            last_publish[0]=rospy.Time.now().to_sec()
    def scan(message):
        try:
            transform=buffer.lookup_transform('map',message.header.frame_id,message.header.stamp,rospy.Duration(.08)).transform
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            failures[0]+=1;return
        q=transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        free=ray_free(points,[transform.translation.x,transform.translation.y,yaw],message.ranges,
                      message.angle_min,message.angle_increment,message.range_min,message.range_max)
        with lock:
            changed=evidence.observe(message.header.stamp.to_sec(),free)
            grid[evidence.rows[evidence.cleared],evidence.cols[evidence.cleared]]=0
            if changed or rospy.Time.now().to_sec()-last_publish[0]>=.5:publish()
    publish()
    rospy.Subscriber('/scan_obstacles',LaserScan,scan,queue_size=1)
    rospy.spin()

if __name__=='__main__':main()
