#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def rotate():
    rospy.init_node('rotate_node')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)  # 10 Hz

    twist = Twist()
    twist.linear.x = 3
    twist.angular.z = 5  # radians/sec (positive → counter‐clockwise)

    rospy.loginfo("Starting rotation...")
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        rotate()
    except rospy.ROSInterruptException:
        pass
