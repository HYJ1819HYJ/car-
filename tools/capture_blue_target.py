#!/usr/bin/env python3
"""Capture camera frames and same-frame blue-target measurements without commanding the robot."""

import datetime
import json
import os
import threading
import time
from collections import OrderedDict, deque

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from stm32_bridge.msg import BlueTargetObservation


OUTPUT_DIR = "/workspace/xiaoche/outputs/blue-target-captures"
LOCK = threading.Lock()
BRIDGE = CvBridge()
last_image = None
last_image_received = 0.0
observations = OrderedDict()
joints = deque(maxlen=30)


def frame_key(header):
    return (header.seq, header.stamp.secs, header.stamp.nsecs)


def image_callback(msg):
    global last_image, last_image_received
    with LOCK:
        last_image = msg
        last_image_received = time.monotonic()


def observation_callback(msg):
    with LOCK:
        observations[frame_key(msg.header)] = msg
        while len(observations) > 100:
            observations.popitem(last=False)


def joint_callback(msg):
    with LOCK:
        joints.append(msg)


def capture():
    with LOCK:
        image = last_image
        received = last_image_received
    if image is None or time.monotonic() - received > 2.0:
        print("当前没有新鲜的摄像头画面，请检查摄像头节点及连接。", flush=True)
        return

    # The detector copies the camera header, so only use an observation from
    # the exact captured frame. Save the original even if detection is absent.
    deadline = time.monotonic() + 1.0
    observation = None
    while time.monotonic() < deadline and not rospy.is_shutdown():
        with LOCK:
            observation = observations.get(frame_key(image.header))
        if observation is not None:
            break
        time.sleep(0.02)

    with LOCK:
        joint = min(joints, key=lambda m: abs((m.header.stamp - image.header.stamp).to_sec())) if joints else None

    frame = BRIDGE.imgmsg_to_cv2(image, "bgr8")
    height, width = frame.shape[:2]
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    base = os.path.join(OUTPUT_DIR, "blue_target_" + stamp)
    raw_path, marked_path, json_path = base + ".png", base + "_marked.png", base + ".json"

    metrics = {
        "camera_stamp": image.header.stamp.to_sec(),
        "image_width_px": width,
        "image_height_px": height,
        "detected": bool(observation.detected) if observation is not None else None,
        "target_contour_area_percent": None,
        "bounding_box_area_percent": None,
        "width_percent": None,
        "height_percent": None,
        "bbox_touches_image_edge": None,
        "gimbal_joint_names": list(joint.name) if joint is not None else None,
        "gimbal_joint_positions_rad": list(joint.position) if joint is not None else None,
        "gimbal_stamp": joint.header.stamp.to_sec() if joint is not None else None,
    }
    marked = frame.copy()
    if observation is not None and observation.detected:
        bw, bh = observation.pixel_width, observation.pixel_height
        x1 = int(round(observation.center_x - bw / 2.0))
        y1 = int(round(observation.center_y - bh / 2.0))
        x2 = int(round(observation.center_x + bw / 2.0))
        y2 = int(round(observation.center_y + bh / 2.0))
        metrics.update({
            "bbox_x_px": x1,
            "bbox_y_px": y1,
            "bbox_width_px": bw,
            "bbox_height_px": bh,
            "target_contour_area_percent": 100.0 * observation.contour_ratio,
            "bounding_box_area_percent": 100.0 * bw * bh / (width * height),
            "width_percent": 100.0 * bw / width,
            "height_percent": 100.0 * bh / height,
            "bbox_touches_image_edge": x1 <= 0 or y1 <= 0 or x2 >= width or y2 >= height,
        })
        cv2.rectangle(marked, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = "contour %.2f%%  box %.0fx%.0f" % (metrics["target_contour_area_percent"], bw, bh)
        cv2.putText(marked, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    else:
        cv2.putText(marked, "No matched blue target", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 128, 255), 2)

    if not cv2.imwrite(raw_path, frame) or not cv2.imwrite(marked_path, marked):
        raise RuntimeError("保存图片失败，请检查输出目录权限和磁盘空间")
    with open(json_path, "w", encoding="utf-8") as handle:
        json.dump(metrics, handle, ensure_ascii=False, indent=2)

    print("已保存：" + os.path.basename(raw_path), flush=True)
    if metrics["detected"]:
        print("蓝纸轮廓占画面面积 %.2f%%；宽度 %.2f%%，高度 %.2f%%；外接框面积 %.2f%%。" % (
            metrics["target_contour_area_percent"], metrics["width_percent"],
            metrics["height_percent"], metrics["bounding_box_area_percent"]), flush=True)
        if metrics["bbox_touches_image_edge"]:
            print("目标框贴到画面边缘，占比可能不完整。", flush=True)
    else:
        print("已保存原图，但该帧没有匹配到蓝纸识别结果，未计算占比。", flush=True)
    print("同时保存了标注图和同名 JSON 数据。", flush=True)


def main():
    rospy.init_node("capture_blue_target", anonymous=True, disable_signals=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback, queue_size=3)
    rospy.Subscriber("/blue_target/observation", BlueTargetObservation, observation_callback, queue_size=10)
    rospy.Subscriber("/gimbal/joint_states", JointState, joint_callback, queue_size=10)
    print("每按一次回车截取一张图；输入 q 后回车退出。", flush=True)
    while not rospy.is_shutdown():
        try:
            command = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if command == "q":
            break
        if command:
            print("请直接按回车截图，或输入 q 后回车退出。", flush=True)
            continue
        try:
            capture()
        except Exception as exc:
            print("截图失败：%s" % exc, flush=True)


if __name__ == "__main__":
    main()
