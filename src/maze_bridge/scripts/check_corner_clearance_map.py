#!/usr/bin/env python3
"""Reject a map-specific clearance mask paired with another or edited map."""
import hashlib
import json
from pathlib import Path
import rospy
import yaml


def validate(source_path, mask_path):
    source = yaml.safe_load(source_path.read_text())
    mask = yaml.safe_load(mask_path.read_text())
    report = json.loads(mask_path.with_suffix('.json').read_text())
    if report['map'] != source_path.name:
        raise ValueError('Corner selections belong to another map')
    for key in ['resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh']:
        if mask[key] != source[key]:
            raise ValueError('Corner mask metadata mismatch: ' + key)
    for meta, path, key in [(source, source_path, 'source_sha256'), (mask, mask_path, 'mask_sha256')]:
        image = path.parent / Path(meta['image']).name
        if hashlib.sha256(image.read_bytes()).hexdigest() != report[key]:
            raise ValueError('Map or mask changed; regenerate corner selections')


if __name__ == '__main__':
    rospy.init_node('check_corner_clearance_map')
    try:
        validate(Path(rospy.get_param('~map_file')), Path(rospy.get_param('~mask_file')))
    except Exception as error:
        rospy.logfatal('Corner clearance mask rejected: %s', error)
        raise SystemExit(1)
    rospy.loginfo('Corner clearance mask matches the saved map')
    rospy.spin()
