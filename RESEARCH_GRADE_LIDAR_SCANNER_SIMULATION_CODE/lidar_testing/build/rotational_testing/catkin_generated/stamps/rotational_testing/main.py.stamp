#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class RotationalTesting:
    def __init__(self):
        rospy.init_node('rotational_testing_node')
        
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Variables
        self.scan_data = None
        self.twist_msg = Twist()
        
        rospy.loginfo("Rotational Testing Node Started")
    
    def scan_callback(self, data):
        """Process lidar scan data"""
        self.scan_data = data
        
        # Example: Find minimum distance
        if len(data.ranges) > 0:
            min_range = min([r for r in data.ranges if r > data.range_min and r < data.range_max])
            rospy.loginfo(f"Minimum range: {min_range:.2f}m")
    
    def rotate_robot(self, angular_velocity=0.5):
        """Make robot rotate"""
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(self.twist_msg)
    
    def stop_robot(self):
        """Stop robot movement"""
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)
    
    def run(self):
        """Main execution loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            if self.scan_data is not None:
                # Example behavior: rotate slowly
                self.rotate_robot(0.2)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        tester = RotationalTesting()
        tester.run()
    except rospy.ROSInterruptException:
        pass
