#!/usr/bin/env python3
"""Show camera, detected target and gimbal state; never publish robot commands."""
import threading

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from stm32_bridge.msg import BlueTargetObservation

rospy.init_node('direct_tracking_view')
lock = threading.Lock()
bridge = CvBridge()
latest = {}


def save(name):
    def callback(msg):
        with lock:
            latest[name] = msg
    return callback


rospy.Subscriber('/usb_cam/image_raw', Image, save('image'), queue_size=1)
rospy.Subscriber('/blue_target/observation', BlueTargetObservation, save('obs'), queue_size=1)
rospy.Subscriber('/gimbal/joint_states', JointState, save('joint'), queue_size=1)
rospy.Subscriber('/blue_autonomy/state', String, save('state'), queue_size=1)
window = 'Blue target tracking - read only'
cv2.namedWindow(window, cv2.WINDOW_NORMAL)
rate = rospy.Rate(25)
while not rospy.is_shutdown():
    with lock:
        data = dict(latest)
    if 'image' in data:
        img = bridge.imgmsg_to_cv2(data['image'], 'bgr8').copy()
        obs = data.get('obs')
        if obs and obs.detected and abs((data['image'].header.stamp - obs.header.stamp).to_sec()) < .4:
            x1 = int(round(obs.center_x - obs.pixel_width / 2))
            y1 = int(round(obs.center_y - obs.pixel_height / 2))
            x2 = int(round(obs.center_x + obs.pixel_width / 2))
            y2 = int(round(obs.center_y + obs.pixel_height / 2))
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = 'Target area: %.2f%% / 16%%' % (100.0 * obs.contour_ratio)
        else:
            label = 'Target: not detected'
        cv2.putText(img, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 0), 2)
        if 'joint' in data:
            joints = dict(zip(data['joint'].name, data['joint'].position))
            if 'camera_pan_joint' in joints:
                label = 'Pan: %.1f deg' % (joints['camera_pan_joint'] * 180.0 / 3.141592653589793)
                cv2.putText(img, label, (8, 47), cv2.FONT_HERSHEY_SIMPLEX, .52, (0, 255, 0), 2)
        if 'state' in data:
            label = data['state'].data.split(':', 1)[0]
            cv2.putText(img, 'State: ' + label, (8, 70), cv2.FONT_HERSHEY_SIMPLEX, .52, (0, 255, 0), 2)
        cv2.imshow(window, img)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
    if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
        break
    rate.sleep()
cv2.destroyAllWindows()
