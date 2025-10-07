#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class LaserVisualizer:
    def __init__(self):
        rospy.init_node('viz_scan')
        # Topic & parameters
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.frame_id   = rospy.get_param('~frame_id', 'base_link')
        self.max_range  = rospy.get_param('~max_range', 8.0)  # meters
        self.color_r    = rospy.get_param('~r', 255.0)
        self.color_g    = rospy.get_param('~g', 223.0)
        self.color_b    = rospy.get_param('~b', 0)
        self.color_a    = rospy.get_param('~a', 0.5)
        self.width      = rospy.get_param('~width', 0.02)

        # Publisher for a single Marker (LINE_LIST)
        self.marker_pub = rospy.Publisher('scan_beams', Marker, queue_size=1)

        # Subscriber
        rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan)
        rospy.loginfo("LiDAR visualizer ready: clipping to %.2fm", self.max_range)
        rospy.spin()

    def cb_scan(self, scan: LaserScan):
        # LIDAR beams as lines
        m = Marker()
        m.header.stamp = scan.header.stamp
        m.header.frame_id = self.frame_id
        m.ns = "lidar_beams"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = self.width  # line width
        m.color.r = self.color_r
        m.color.g = self.color_g
        m.color.b = self.color_b
        m.color.a = self.color_a

        angle = scan.angle_min
        for r in scan.ranges:
            dist = min(r, self.max_range) if r > 0 else self.max_range
            x = dist * math.cos(angle)
            y = dist * math.sin(angle)
            m.points.append(Point(0.0, 0.0, 0.0))
            m.points.append(Point(x, y, 0.0))
            angle += scan.angle_increment

        self.marker_pub.publish(m)

        # Add a perfect circle at max_range
        circle = Marker()
        circle.header.stamp = scan.header.stamp
        circle.header.frame_id = self.frame_id
        circle.ns = "lidar_circle"
        circle.id = 1
        circle.type = Marker.LINE_STRIP
        circle.action = Marker.ADD
        circle.scale.x = self.width
        circle.color.r = 1.0
        circle.color.g = 0.0
        circle.color.b = 0.0
        circle.color.a = 0.5

        num_points = 100
        for i in range(num_points + 1):
            theta = 2 * math.pi * i / num_points
            x = self.max_range * math.cos(theta)
            y = self.max_range * math.sin(theta)
            circle.points.append(Point(x, y, 0.0))

        self.marker_pub.publish(circle)

if __name__ == '__main__':
    try:
        LaserVisualizer()
    except rospy.ROSInterruptException:
        pass
