#!/usr/bin/env python3
"""Generate/reuse a matching map overlay, then publish the persistent mask."""
import os
from pathlib import Path
import numpy as np
from PIL import Image
import rospy
import yaml
from nav_msgs.msg import OccupancyGrid
from maze_bridge.corner_clearance import image_path
from maze_bridge.corner_clearance_cache import prepare, validate


def main():
    rospy.init_node('corner_clearance_map')
    source = Path(rospy.get_param('~map_file'))
    cache = rospy.get_param('~cache_directory',
                           str(Path(os.environ.get('ROS_HOME', '~/.ros')).expanduser()/'corner_clearance'))
    mask, report, action = prepare(
        source, cache, explicit_mask=rospy.get_param('~mask_file', '') or None,
        extra=float(rospy.get_param('~extra_clearance', .05)),
        length=float(rospy.get_param('~minimum_wall_length', .25)),
        angle=float(rospy.get_param('~minimum_turn_degrees', 45.)),
        bounds=rospy.get_param('~bounds', []) or None,
        scope=rospy.get_param('~scope', 'main_walls'),
        endpoints=bool(rospy.get_param('~include_wall_endpoints', True)),
        endpoint_radius=float(rospy.get_param('~wall_endpoint_radius', .05)))
    meta = yaml.safe_load(mask.read_text())
    pixels = np.flipud(np.array(Image.open(image_path(mask, meta)).convert('L')))
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.header.stamp = message.info.map_load_time = rospy.Time.now()
    message.info.resolution = meta['resolution']
    message.info.height, message.info.width = pixels.shape
    message.info.origin.position.x, message.info.origin.position.y = meta['origin'][:2]
    message.info.origin.orientation.w = 1.
    message.data = np.where(pixels == 0, 100, -1).flatten().tolist()
    publisher = rospy.Publisher('/corner_clearance_map', OccupancyGrid, queue_size=1, latch=True)
    publisher.publish(message)
    rospy.set_param('~resolved_mask_file', str(mask))
    rospy.set_param('~corner_count', report.get('corner_count', len(report['selections'])))
    rospy.set_param('~endpoint_count', report.get('endpoint_count', 0))
    rospy.set_param('~preparation', action)
    rospy.loginfo('Wall protection %s: %d corners + %d free ends; corner margin %.3f m, end radius %.3f m; source %s',
                  action, report.get('corner_count', len(report['selections'])), report.get('endpoint_count', 0),
                  report['extra_clearance'], report.get('wall_endpoint_radius', .05), source)
    if not report['selections']:
        rospy.logwarn('No corners or wall ends meet the thresholds; published an empty additive mask')
    failures = []
    def recheck(_):
        try:
            validate(source, mask)
        except Exception as error:
            failures.append(str(error))
            rospy.logfatal('Corner map changed during navigation: %s', error)
            rospy.signal_shutdown('Map/mask pairing changed')
    timer = rospy.Timer(rospy.Duration(2.), recheck)
    rospy.spin()
    timer.shutdown()
    if failures:
        raise RuntimeError(failures[-1])


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        rospy.logfatal('Cannot prepare corner protection: %s', error)
        raise SystemExit(1)
