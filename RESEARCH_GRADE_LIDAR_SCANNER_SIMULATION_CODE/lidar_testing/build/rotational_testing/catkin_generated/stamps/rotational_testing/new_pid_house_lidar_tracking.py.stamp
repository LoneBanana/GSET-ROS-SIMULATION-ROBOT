#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.signal import medfilt, find_peaks
from enhanced_pid import EnhancedPIDController

class Navigator:
    def __init__(self):
        rospy.init_node('dijkstra_navigator', anonymous=True)

        # Parameters
        self.goal_x = rospy.get_param('~goal_x', 3.0)
        self.goal_y = rospy.get_param('~goal_y', 3.0)
        self.distance_threshold = 0.1
        self.scan_processed = False
        self.is_rotating = False
        self.current_state = "INIT"

        # Corner detection parameters
        self.min_corner_distance = 0
        self.max_corner_distance = 8
        self.peak_threshold = 0.2
        self.window_size = 5
        self.median_window = 5
        self.gradient_threshold = 0.15
        self.peak_prominence = 0.1
        self.min_edge_length = 1.0
        self.edge_deviation_threshold = 0.1
        self.edge_waypoint_spacing = 0.5
        self.los_resolution = 0.1
        self.los_obstacle_threshold = 0.3
        self.robot_width = 0.178
        self.robot_safety_margin = 0.05
        self.robot_radius = (self.robot_width / 2) + self.robot_safety_margin
        self.rotation_count = 0
        self.max_rotations_before_edge_fallback = 10
        self.edge_fallback_active = False

        # Subscribers & Publishers
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', MarkerArray, queue_size=10)

        # Robot state
        self.current_pose = None
        self.start_pose = None
        self.corners = []
        self.edges = []
        self.nodes = []
        self.target_node = None
        self.scan_angles = []
        self.scan_ranges = []
        self.scan_pose = None
        self.latest_scan_ranges = None
        self.latest_scan_angles = None
        self.scan_angle_min = None
        self.scan_angle_max = None
        self.scan_angle_increment = None

        # Initialize PID controller
        self.pid_controller = EnhancedPIDController(autotune=False)

        # Wait for initial pose
        rospy.loginfo("Waiting for initial pose...")
        while not rospy.is_shutdown() and self.current_pose is None:
            rospy.sleep(0.1)
        self.start_pose = self.current_pose
        self.start_yaw = self.start_pose[2]
        rospy.loginfo(f"Start position: {self.start_pose}")

        # Step 1: Perform initial 360° scan
        self.current_state = "ROTATING"
        self.start_rotation()

    def start_rotation(self):
        self.is_rotating = True
        self.scan_processed = False
        self.rotation_start_time = rospy.Time.now()
        self.scan_angles = []
        self.scan_ranges = []
        self.scan_pose = self.current_pose

        twist = Twist()
        twist.angular.z = 0.3
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Starting 360° scan rotation...")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        self.current_pose = (x, y, yaw)

        # Check if rotation should stop
        if self.current_state == "ROTATING":
            rotation_duration = (rospy.Time.now() - self.rotation_start_time).to_sec()
            if rotation_duration >= (2 * np.pi / 0.3):
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.is_rotating = False
                self.current_state = "PROCESSING_SCAN"
                rospy.loginfo("360° scan complete. Ready to process scan.")

    # ... (other methods remain the same until move_to_node) ...

    def run_dijkstra(self):
        goal = self.nodes[-1]
        goal_pos = (goal[0], goal[1])

        min_dist = float('inf')
        next_node_idx = 0

        rospy.loginfo("\nNode Evaluation:")
        rospy.loginfo("Index | Coordinates    | Type    | Distance to Goal | Current to Node | Total Weight")
        rospy.loginfo("---|---|---|---|---|")

        for i, node in enumerate(self.nodes[:-1]):
            node_pos = (node[0], node[1])
            dist = np.linalg.norm(np.array(node_pos) - np.array(goal_pos))
            current_to_node = np.linalg.norm(np.array(node_pos) - np.array(self.current_pose[:2]))
            weight = dist * (1 + 0.2 * current_to_node / max(dist, 0.1))

            rospy.loginfo(f"{i:5d} | ({node[0]:.2f}, {node[1]:.2f}) | {node[2]:<10} | {dist:16.2f} | {current_to_node:15.2f} | {weight:12.2f}")
            if weight < min_dist:
                min_dist = weight
                next_node_idx = i

        rospy.loginfo(f"Selected target: Node {next_node_idx} at ({self.nodes[next_node_idx][0]:.2f}, {self.nodes[next_node_idx][1]:.2f}) with total weight {min_dist:.2f}")
        return next_node_idx

    def move_to_node(self):
        if self.target_node is None or self.current_pose is None:
            rospy.logwarn("No target node or current pose. Cannot move.")
            return

        target_x, target_y, _ = self.target_node
        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        distance_error = math.sqrt(dx**2 + dy**2)

        if distance_error > self.distance_threshold:
            # Calculate target yaw
            target_yaw = math.atan2(dy, dx)
            
            # Update PID controller with current position and target
            self.pid_controller.set_target(target_x, target_y, target_yaw)
            cmd = self.pid_controller.update(
                self.current_pose[0],
                self.current_pose[1],
                self.current_pose[2]
            )
            self.cmd_vel_pub.publish(cmd)
        else:
            # Reached the node
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo("Reached node. Starting next scan...")
            self.current_state = "ROTATING"
            self.start_rotation()

    def main_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.current_state == "MOVING":
                self.move_to_node()
            rate.sleep()

    # ... (rest of the methods remain the same) ...

if __name__ == '__main__': 
    try:
        navigator = Navigator()
        navigator.main_loop()
    except rospy.ROSInterruptException:
        pass