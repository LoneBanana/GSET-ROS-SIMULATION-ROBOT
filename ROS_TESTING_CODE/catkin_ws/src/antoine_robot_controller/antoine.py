#!/usr/bin/env python3

#Need to fix
import rospy
from geometry_msgs.msg import Twist
import os
import csv
from datetime import datetime

def antoine_controller(x_d, y_d, yaw_d):
    msg = Twist()
    msg.linear.x = x_d
    msg.linear.y = y_d
    msg.angular.z = yaw_d
    return msg #In case we decide to do something with publishers

def ending():
    csv_file.close()
    rospy.loginfo("CSV file closed successfully.")

if __name__ == "__main__":
    rospy.init_node("antoine_robot_controller")
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)

    data_folder = "/home/lonebanana/catkin_ws/record_data"
    os.makedirs(data_folder, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(data_folder, f"antoine_data_{timestamp}.csv")
    csv_file = open(csv_path, mode="w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["time", "x_d", "y_d", "yaw_d"])
    
    rospy.on_shutdown(ending)
    
    while not rospy.is_shutdown():
        message = antoine_controller(5, 0.0, 0.0)
        vel_pub.publish(message)
        
        csv_writer.writerow([
            f"{rospy.get_time():.3f}",
            f"{message.linear.x:.6f}",
            f"{message.linear.y:.6f}",
            f"{message.angular.z:.6f}"
        ])
        csv_file.flush()

        rate.sleep()
