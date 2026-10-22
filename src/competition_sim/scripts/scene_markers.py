#!/usr/bin/env python3
"""Show ground markings and original CAD without adding lidar obstacles."""
import json
from pathlib import Path
import rospkg
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def main():
    rospy.init_node('competition_scene')
    pkg = Path(rospkg.RosPack().get_path('competition_sim'))
    geometry = json.loads((pkg / 'maps/geometry.json').read_text())
    mesh = pkg / 'models/wheel_hub_car_mm.stl'
    if not mesh.exists():
        mesh = pkg.parents[1] / 'models/wheel_hub_car_mm.stl'
    pub = rospy.Publisher('/simulation/scene', MarkerArray, queue_size=1, latch=True)
    markers = []
    def marker(kind, frame='map'):
        m = Marker()
        m.header.frame_id = frame
        m.ns, m.id, m.type, m.action = 'competition', len(markers), kind, Marker.ADD
        m.pose.orientation.w = 1
        m.color.a = 1
        markers.append(m)
        return m
    for key, text in [('start_square_m', 'START 0.8 x 0.8 m'),
                      ('finish_square_m', 'FINISH (example position)')]:
        (x0, y0), (x1, y1) = geometry[key]
        m = marker(Marker.LINE_STRIP)
        m.scale.x = 0.025
        m.color.r, m.color.g = 1, 0.8
        m.points = [Point(x, y, 0.03) for x, y in
                    [(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)]]
        m = marker(Marker.TEXT_VIEW_FACING)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (x0+x1)/2, y1+0.16, 0.1
        m.scale.z, m.text = 0.15, text
        m.color.r, m.color.g, m.color.b = 0.15, 0.6, 1
    # The black lines are paint, not occupancy or range returns.
    for points in [[(4.627,6.915),(8.8,6.915)], [(0.4,0),(0.4,3.025)]]:
        m = marker(Marker.LINE_STRIP)
        m.scale.x = 0.025
        m.points = [Point(x,y,0.02) for x,y in points]
    m = marker(Marker.MESH_RESOURCE, 'base_footprint')
    m.mesh_resource = mesh.as_uri()
    m.mesh_use_embedded_materials = False
    m.scale.x = m.scale.y = m.scale.z = 0.001
    m.pose.position.x, m.pose.position.y, m.pose.position.z = -0.1517208, -0.245, -0.491
    m.color.r, m.color.g, m.color.b = 0.2, 0.65, 0.95
    scenario = json.loads(Path(rospy.get_param('~scenario_file')).read_text())
    for box in scenario['boxes']:
        m = marker(Marker.CUBE)
        m.pose.position.x,m.pose.position.y = box['center']
        m.pose.position.z = box['size'][2]/2
        m.scale.x,m.scale.y,m.scale.z = box['size']
        m.color.r,m.color.g,m.color.b = box['color']
        m = marker(Marker.TEXT_VIEW_FACING)
        m.pose.position.x,m.pose.position.y = box['center']
        m.pose.position.y += box['size'][1]/2+.15
        m.pose.position.z,m.scale.z = .1,.13
        m.text = 'Relocated box '+str(box['id'])
        m.color.r,m.color.g,m.color.b = box['color']
    for box in scenario.get('mapped_boxes',[]):
        m = marker(Marker.LINE_STRIP)
        m.scale.x = .02
        m.color.r,m.color.g,m.color.b = 1,.1,.1
        m.points = [Point(x,y,.035) for x,y in box['corners']+[box['corners'][0]]]
        m = marker(Marker.TEXT_VIEW_FACING)
        m.pose.position.x,m.pose.position.y = box['center']
        m.pose.position.z,m.scale.z = .04,.12
        m.text = 'Old position '+str(box['id'])
        m.color.r,m.color.g,m.color.b = 1,.1,.1
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        for m in markers:
            m.header.stamp = rospy.Time.now()
        pub.publish(MarkerArray(markers))
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
