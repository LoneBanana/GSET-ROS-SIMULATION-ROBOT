#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math as m

#All imported visualization libraries need to be mentioned in package.xml
#if not already


def scan_callback(data):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lidar_points"
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.scale.x = 0.05  # Size of each point
    marker.scale.y = 0.05
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    yaw = data.angle_min
    for r in data.ranges:
        if data.range_min < r < data.range_max:
            p = Point()
            p.x = r * m.cos(yaw)
            p.y = r * m.sin(yaw)
            p.z = 0
            marker.points.append(p)
        yaw += data.angle_increment
    marker_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("main_viz", anonymous=False)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()