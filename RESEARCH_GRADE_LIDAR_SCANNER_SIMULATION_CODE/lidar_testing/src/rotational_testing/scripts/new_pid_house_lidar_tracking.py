#!/usr/bin/env python3

#shebang strikes once again!
import rospy
import math
import numpy as np
import time
from collections import deque
import threading
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry as NavOdometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.signal import medfilt, find_peaks
import csv
import matplotlib.pyplot as plt
import os
from datetime import datetime



class EnhancedPID:
    def __init__(self, kp, ki, kd, max_out, integ_window=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral_queue = deque(maxlen=integ_window)
        self.prev_error = 0.0
        self.prev_time = None
        self.current_state = "INIT"

    def reset(self):
        self.integral_queue.clear()
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error):
        now = rospy.Time.now().to_sec()
        dt = now - self.prev_time if self.prev_time is not None else 0.1
        self.prev_time = now
        clamped_error = max(-1.0, min(1.0, error))
        
        #proportionl term
        P = self.kp * clamped_error

        if abs(clamped_error) < 0.5:
            self.integral_queue.append(clamped_error * dt)
        
        # integral term
        I = self.ki * sum(self.integral_queue)

        # derivative term
        D = self.kd * (clamped_error - self.prev_error) / dt if dt > 0 else 0.0

        u = P + I + D #PID!

        u_sat = max(-self.max_out, min(self.max_out, u)) #prevent wind-up

        if abs(u) > abs(u_sat) and dt > 0 and len(self.integral_queue) > 0:
            self.integral_queue.pop()

        self.prev_error = clamped_error
        return u_sat

class ZeiglerNicholsTuner:
    def __init__(self, get_control_signal, set_gain, target_step, sample_rate=50):
        self.get_control_signal = get_control_signal
        self.set_gain = set_gain
        self.target_step = target_step
        self.sample_rate = sample_rate

    def find_ultimate_parameters(self):
        Ku = None
        Tu = None
        kp = 0.1
        oscillation_count = 0
        last_times = []
        self.set_gain(kp, 0.0, 0.0)
        start_time = time.time()
        errors = []
        max_kp = 5.0

        while kp < max_kp and oscillation_count < 6:
            error = self.get_control_signal()
            errors.append((time.time(), error))
            if len(errors) > 2:
                _, e1 = errors[-3]
                _, e2 = errors[-2]
                _, e3 = errors[-1]
                if ((e1 < e2 > e3 and abs(e2) > 0.05) or (e1 > e2 < e3 and abs(e2) > 0.05)):
                    oscillation_count += 1
                    last_times.append(time.time())
            kp *= 1.2
            self.set_gain(kp, 0.0, 0.0)
            rospy.sleep(1.0 / self.sample_rate)
        if oscillation_count >= 4 and len(last_times) > 2:
            periods = [last_times[i] - last_times[i-1] for i in range(1, len(last_times))]
            Tu = sum(periods) / len(periods)
            Ku = kp / 1.2
            rospy.loginfo(f"Found Ku={Ku:.3f}, Tu={Tu:.3f}")
        else:
            rospy.logwarn("Tuning failed: Using conservative defaults")
            Ku = 1.5
            Tu = 2.0
        return Ku, Tu

class EnhancedBasePIDNode:
    def __init__(self, x_target, y_target, yaw_target):
        pd = rospy.get_param('~pid_distance', [0.8, 0.01, 0.15, 0.26])
        py = rospy.get_param('~pid_yaw', [1.5, 0.005, 0.1, 1.5])
        self.pid_dist = EnhancedPID(*pd)
        self.pid_yaw = EnhancedPID(*py)
        self.autotune = rospy.get_param('~autotune', False)
        self.x_target, self.y_target, self.yaw_target = x_target, y_target, yaw_target
        self.x_actual = self.y_actual = self.yaw_actual = 0.0

        self.slow_speed_thresh = rospy.get_param('~slow_speed_thresh', 0.05)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.22)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.84)

        self.odom_sub = rospy.Subscriber('/odom', NavOdometry, self.odom_cb)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.pid_dist.prev_time = rospy.Time.now().to_sec()
        self.pid_yaw.prev_time = rospy.Time.now().to_sec()

        if self.autotune:
            self.run_tuner()
        self.rate = rospy.Rate(20)

        self.should_stop = False
        self.is_running = False

    def odom_cb(self, msg):
        self.x_actual = msg.pose.pose.position.x
        self.y_actual = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.yaw_actual = yaw

    def imu_cb(self, msg):
        q = msg.orientation
        _, _, yaw_imu = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_actual = yaw_imu

    def stop_controller(self):
        self.should_stop = True
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        rospy.loginfo("PID controller manually stopped")

    def get_current_error(self):
        #getting current postition and yaw errors
        dx = self.x_target - self.x_actual
        dy = self.y_target - self.y_actual
        position_error = math.sqrt(dx**2 + dy**2)
        raw_yaw_error = self.yaw_target - self.yaw_actual
        yaw_error = math.atan2(math.sin(raw_yaw_error), math.cos(raw_yaw_error))
        return position_error, abs(yaw_error)

    def run_tuner(self):
        def get_err():
            dx = self.x_target - self.x_actual
            dy = self.y_target - self.y_actual
            return dx * math.cos(self.yaw_actual) + dy * math.sin(self.yaw_actual)
        
        def set_gains(kp, ki, kd):
            self.pid_dist.kp, self.pid_dist.ki, self.pid_dist.kd = kp, ki, kd

        tuner = ZeiglerNicholsTuner(get_err, set_gains, self.x_target)
        Ku, Tu = tuner.find_ultimate_parameters()

        Kp = 0.4 * Ku
        Ki = 0.8 * Ku / Tu if Tu > 0 else 0.0
        Kd = 0.05 * Ku * Tu
        
        rospy.loginfo(f"Tuned gains: Kp={Kp:.3f}, Ki={Ki:.3f}, Kd={Kd:.3f}")
        self.pid_dist.kp, self.pid_dist.ki, self.pid_dist.kd = Kp, Ki, Kd

    def run_controller(self):
        self.should_stop = False
        self.is_running = True
        
        while not rospy.is_shutdown() and not self.should_stop:
            dx = self.x_target - self.x_actual
            dy = self.y_target - self.y_actual
            distance = math.sqrt(dx**2 + dy**2)
            forward_err = dx * math.cos(self.yaw_actual) + dy * math.sin(self.yaw_actual)
            heading = math.atan2(dy, dx)
            yaw_err_heading = math.atan2(
                math.sin(heading - self.yaw_actual),
                math.cos(heading - self.yaw_actual)
            )
            v = self.pid_dist.update(forward_err)
            if distance > 0.1:
                psi_err = yaw_err_heading
            else:
                raw = self.yaw_target - self.yaw_actual
                psi_err = math.atan2(math.sin(raw), math.cos(raw))
            omega = self.pid_yaw.update(psi_err)
            abs_v = abs(v)
            abs_omega = abs(omega)
            if abs_v < 0.05:
                omega *= 0.7
            if abs_v < 0.01 and abs_omega > 0.5:
                v = 0.01 * (1 if v >= 0 else -1)
            v = max(-self.max_linear_speed, min(self.max_linear_speed, v))
            omega = max(-self.max_angular_speed, min(self.max_angular_speed, omega))
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = omega
            self.cmd_pub.publish(cmd)
            self.rate.sleep()
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.is_running = False
        rospy.loginfo("PID controller stopped")

class Navigator:
    def __init__(self):
        rospy.init_node('dijkstra_navigator', anonymous=True)
        
        # Parameters
        self.goal_x = rospy.get_param('~goal_x', 3.0) #x-coord of goal set to 3 meters from origin along x-axis
        self.goal_y = rospy.get_param('~goal_y', 3.0) #y-coord of goal set to 3 meters from origin along y-axis
        self.rotation_speed = 0.3  # rad/s
        self.distance_threshold = 0.1
        self.scan_processed = False
        self.is_rotating = False


        # information on the different states of the TurtleBot3 Burger
        #     "INIT", "ROTATING", "PROCESSING_SCAN", "MOVING"
        self.current_state = "INIT"
        
        # Corner detection parameters
        self.min_corner_distance = 0  # meters --> depending on the configured goal_point, increasing may yield more optimal results
        self.max_corner_distance = 8    # meters --> range of LDS-02 LiDAR scanner --> 8 meters
        self.window_size = 5 
        self.median_window = 5
#NOTE: optimization - increasing min_corner_distance may yield more optimal results

        #detection of corners using gradients
        self.gradient_threshold = 0.15    # minimum gradient magnitude for corner (m/rad)
        self.peak_prominence = 0.1        # how much peak should stand out (prominence) from neightbhors (m/rad)
        #self.peak_prominence can decrease sensitivity to noise
        
#NOTE: optimization - increasing peak_prominence may yield drastically different results, allowing for more optimzal edge detection
#NOTE: optimization - increasing gradient_threshold may yield more optimal results, but may also result in fewer corners being detected
#NOTE: optimization - decreasing gradient_threshold may increase the number of conrers being detected --> if map is very large (within self.max_corner_distance), increasing this may become necessity

        # edge detection params
        self.min_edge_length = 1 # min edge length (m)
        self.edge_deviation_threshold = 0.1  # max deviation from straight line (m)
        self.edge_waypoint_spacing = 0.5     # spacing in between waypoints on edges (m)

#NOTE: optimization - increase min_edge_length for greater points being detected based on length of edge
#NOTE: edge_deviation_threshold - decreasing may allow for greater allowable points
#NOTE: edge_waypoint_spacing - increasing yields greater points allong edges, increasing allowable points to travel toward
        
        #data-collection initialization, declaration, and set-up
        self.data_collection_enabled = True
        self.start_time = None
        self.data_log = []
        self.starting_position = None
        self.total_distance_traveled = 0
        self.last_position = None
        self.world_name = rospy.get_param('~world_name', 'unknown_world')  # Allow world name parameter
        self.data_directory = "/home/lonebanana/lidar_testing/src/research_storage_data"

        if not os.path.exists(self.data_directory):
            os.makedirs(self.data_directory)
        
        self.los_resolution = 0.1  # meters between ray-casting points
        self.los_obstacle_threshold = 0.3  # minimum distance to obstacle for clear path
        
        self.robot_width = 0.178  # meters
        self.robot_safety_margin = 0.05
        self.robot_safety_multiplier = 2
        self.robot_radius = (self.robot_width / 2) + self.robot_safety_margin
        self.robot_radius *= self.robot_safety_multiplier
#NOTE: robot_safety_multiplier may be optimized --> lowering will lead to more point selection
#NOTE: robot_safety_margin may also be optimized --> decreasing will lead to more point selection
        
        # edge-fallback mechanism --> after 10 rotations, if no nodes are found, will treat edges as nodes
        self.rotation_count = 0
        self.max_rotations_before_edge_fallback = 10
        self.edge_fallback_active = False # if True, will use edge detection fallback mechanism
        

        self.position_error_threshold = rospy.get_param('~position_error_threshold', 0.08)  # meters
        self.yaw_error_threshold = rospy.get_param('~yaw_error_threshold', 0.1)  # radians
        self.max_movement_time = rospy.get_param('~max_movement_time', 30.0)  # seconds
        self.error_check_rate = rospy.Rate(10)  # Check errors at 10Hz
        
        self.current_pid_controller = None
        
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', NavOdometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', MarkerArray, queue_size=10)
        
        # current state of robot
        self.current_pose = None
        self.start_pose = None
        self.corners = []
        self.edges = []
        self.nodes = []
        self.target_node = None
        self.scan_angles = []
        self.scan_ranges = []
        self.scan_pose = None
        
        # dict --> figure-generation
        self.latest_scan_ranges = None
        self.latest_scan_angles = None
        self.scan_angle_min = None
        self.scan_angle_max = None
        self.scan_angle_increment = None
        
        rospy.loginfo("waiting for initial pose...")
        while not rospy.is_shutdown() and self.current_pose is None:
            rospy.sleep(0.1)
        self.start_pose = self.current_pose
        self.start_yaw = self.start_pose[2]
        rospy.loginfo(f"Start position: {self.start_pose}")
        # Initialize data collection
        if self.data_collection_enabled:
            self.start_time = rospy.Time.now().to_sec()
            self.starting_position = self.current_pose
            self.last_position = self.current_pose
            rospy.loginfo(f"Data collection initialized for world: {self.world_name}")
        
        # start 2*pi (rad) scan (LiDAR sweep)
        self.current_state = "ROTATING"
        self.start_rotation()
        rospy.on_shutdown(self.shutdown_handler)


    def start_rotation(self):
        """Start the 360-degree rotation for scanning"""
        self.is_rotating = True
        self.scan_processed = False
        self.rotation_start_time = rospy.Time.now()
        self.scan_angles = []
        self.scan_ranges = []
        self.scan_pose = self.current_pose
        
        twist = Twist()
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_pub.publish(twist)
        self.collect_data_point(twist)
        rospy.loginfo("Starting 2(pi) rad scan rotation...")
    
    def odom_callback(self, msg):
        #from robot current pose (/odom topic), corrects position based on offset
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (x, y, yaw)

        if self.current_state == "ROTATING":
            rotation_duration = (rospy.Time.now() - self.rotation_start_time).to_sec()
            if rotation_duration >= (2 * np.pi / self.rotation_speed):
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.cmd_vel_pub.publish(twist)
                self.collect_data_point(twist)
                self.is_rotating = False
                self.current_state = "PROCESSING_SCAN"
                rospy.loginfo("2(pi) rad scan complete. Ready to process scan.")

        self.collect_data_point()
    
    def has_clear_path(self, start_x, start_y, end_x, end_y):
        """Check if there's a clear path between two points using LiDAR data, considering robot width"""
        if self.latest_scan_ranges is None or self.latest_scan_angles is None:
            rospy.logwarn("No scan data available for line-of-sight validation")
            return False
        

        path_distance = np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        path_angle = np.arctan2(end_y - start_y, end_x - start_x)

        perp_angle = path_angle + np.pi/2
        

        num_checks = max(int(path_distance / self.los_resolution), 1)
        

        for i in range(1, num_checks + 1):
            t = i / num_checks
            center_x = start_x + t * (end_x - start_x)
            center_y = start_y + t * (end_y - start_y)
            
            width_check_points = [-self.robot_radius, -self.robot_radius/2, 0, 
                                self.robot_radius/2, self.robot_radius]
            
            for width_offset in width_check_points:

                check_x = center_x + width_offset * np.cos(perp_angle)
                check_y = center_y + width_offset * np.sin(perp_angle)
                rel_x = check_x - self.scan_pose[0]
                rel_y = check_y - self.scan_pose[1]
                cos_yaw = np.cos(-self.scan_pose[2])
                sin_yaw = np.sin(-self.scan_pose[2])
                local_x = rel_x * cos_yaw - rel_y * sin_yaw
                local_y = rel_x * sin_yaw + rel_y * cos_yaw
                check_range = np.sqrt(local_x**2 + local_y**2)
                check_angle = np.arctan2(local_y, local_x)
                check_angle = (check_angle + 2 * np.pi) % (2 * np.pi)
                closest_ray_idx = self.find_closest_ray_index(check_angle)
                if closest_ray_idx is not None:
                    measured_range = self.latest_scan_ranges[closest_ray_idx]
                    if measured_range < check_range:
                        rospy.logdebug(f"Path blocked at ({check_x:.2f}, {check_y:.2f}) "
                                     f"width_offset={width_offset:.2f}: "
                                     f"measured={measured_range:.2f}m, needed={check_range:.2f}m")
                        return False
        
        return True
    
    def find_closest_ray_index(self, target_angle):
        if self.latest_scan_angles is None:
            return None
        angle_diffs = np.abs(self.latest_scan_angles - target_angle)
        angle_diffs_wrapped = np.abs(self.latest_scan_angles - target_angle + 2*np.pi)
        angle_diffs_wrapped2 = np.abs(self.latest_scan_angles - target_angle - 2*np.pi)
        angle_diffs = np.minimum(angle_diffs, np.minimum(angle_diffs_wrapped, angle_diffs_wrapped2))
        closest_idx = np.argmin(angle_diffs)
        if np.isfinite(self.latest_scan_ranges[closest_idx]) and self.latest_scan_ranges[closest_idx] > 0:
            return closest_idx
        return None
    
    def detect_edges(self, ranges, angles):
        #detects straight line segments in LiDAR data
        edges = []
        
        # Filter valid data points
        valid_indices = np.where((ranges > self.min_corner_distance) & 
                                 (ranges < self.max_corner_distance))[0]
        if len(valid_indices) < 10:
            return edges
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        if len(valid_ranges) > self.median_window:
            valid_ranges = medfilt(valid_ranges,kernel_size=self.median_window)
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)
        window_size = 15
        i = 0
        while i < len(valid_ranges) - window_size:
            x_window = x_points[i:i+window_size]
            y_window = y_points[i:i+window_size]
            if len(x_window) < 3:
                i += 1
                continue
            try:
                A = np.vstack([x_window, np.ones(len(x_window))]).T
                m, b = np.linalg.lstsq(A, y_window, rcond=None)[0]
                line_y = m * x_window + b
                deviations = np.abs(y_window - line_y)
                max_deviation = np.max(deviations)
                if max_deviation < self.edge_deviation_threshold:
                    edge_length = np.sqrt((x_window[-1] - x_window[0])**2 + 
                                        (y_window[-1] - y_window[0])**2)
                    
                    if edge_length > self.min_edge_length:
                        start_angle = valid_angles[i]
                        end_angle = valid_angles[i + window_size - 1]
                        start_range = valid_ranges[i]
                        end_range = valid_ranges[i + window_size - 1]
                        edges.append({
                            'start_range': start_range,
                            'start_angle': start_angle,
                            'end_range': end_range,
                            'end_angle': end_angle,
                            'length': edge_length,
                            'slope': m,
                            'intercept': b
                        })
                        i += window_size // 2
                    else:
                        i += 1
                else:
                    i += 1 
            except np.linalg.LinAlgError:
                i += 1
        return edges
    def detect_corners(self, ranges, angles):
        #detects straight line segments in LiDAR data
        valid_indices = np.where((ranges > self.min_corner_distance) & 
                                 (ranges < self.max_corner_distance))[0]
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        if len(valid_ranges) > self.median_window:
            valid_ranges = medfilt(valid_ranges, kernel_size=self.median_window)
        gradient = np.gradient(valid_ranges)
        abs_gradient = np.abs(gradient)
        peaks, _ = find_peaks(abs_gradient, 
                             height=self.gradient_threshold,
                             prominence=self.peak_prominence)
        corners = []
        for i in peaks:
            if i == 0 or i == len(valid_ranges)-1:
                continue
            if gradient[i] < 0:
                corners.append((valid_ranges[i], valid_angles[i], 'concave'))
            else:
                corners.append((valid_ranges[i], valid_angles[i], 'convex'))
        
        rospy.loginfo(f"Derivative-based detection found {len(corners)} corners")
        return corners
    
    def convert_edges_to_corners(self, edges):
        #converts detected edges to nodes (when edges are absolutely necessary)
        edge_corners = []
        for edge in edges:
            start_x = edge['start_range'] * np.cos(edge['start_angle'])
            start_y = edge['start_range'] * np.sin(edge['start_angle'])
            end_x = edge['end_range'] * np.cos(edge['end_angle'])
            end_y = edge['end_range'] * np.sin(edge['end_angle'])
            num_waypoints = max(2, int(edge['length'] / self.edge_waypoint_spacing))
            for i in range(1, num_waypoints):
                t = i / num_waypoints
                wp_x = start_x + t * (end_x - start_x)
                wp_y = start_y + t * (end_y - start_y)
                wp_range = np.sqrt(wp_x**2 + wp_y**2)
                wp_angle = np.arctan2(wp_y, wp_x)
                edge_corners.append((wp_range, wp_angle, 'edge'))
        return edge_corners
    
    def scan_callback(self, scan_msg):
        #uses LiDAR for corner detection and data collection implementation
        # Always update latest scan data for line-of-sight checks
        if self.current_state != "PROCESSING_SCAN" or self.scan_processed:
            return
        rospy.loginfo("Processing scan data")
        ranges = np.array(scan_msg.ranges)
        raw_angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        # store scan data for line-of-sight validation
        self.latest_scan_ranges = ranges.copy()
        self.latest_scan_angles = raw_angles.copy()
        self.scan_angle_min = scan_msg.angle_min
        self.scan_angle_max = scan_msg.angle_max
        self.scan_angle_increment = scan_msg.angle_increment
        if self.scan_pose is None:
            rospy.logwarn("No scan pose recorded! Using current pose.")
            self.scan_pose = self.current_pose
        angles = (raw_angles + self.scan_pose[2]) % (2 * np.pi)
        self.corners = self.detect_corners(ranges, angles)
        rospy.loginfo(f"Detected {len(self.corners)} corners")
        if not self.edge_fallback_active and self.rotation_count >= self.max_rotations_before_edge_fallback:
            rospy.logwarn(f"Activating edge fallback after {self.rotation_count} rotations without viable corners")
            self.edge_fallback_active = True
        edge_corners = []
        if self.edge_fallback_active:
            self.edges = self.detect_edges(ranges, raw_angles)
            rospy.loginfo(f"Edge fallback mode: Detected {len(self.edges)} edges")
            edge_corners = self.convert_edges_to_corners(self.edges)
            rospy.loginfo(f"Edge fallback mode: Generated {len(edge_corners)} edge-based corners")
            all_corners = self.corners + edge_corners
        else:
            all_corners = self.corners
        self.nodes = []
        valid_nodes_count = 0
        
        for r, theta, corner_type in all_corners:
            new_r = r / 2.0
            global_x = self.scan_pose[0] + new_r * np.cos(theta)
            global_y = self.scan_pose[1] + new_r * np.sin(theta)
            if self.has_clear_path(self.current_pose[0], self.current_pose[1], global_x, global_y):
                self.nodes.append((global_x, global_y, corner_type))
                valid_nodes_count += 1
                rospy.logdebug(f"Valid node: ({global_x:.2f}, {global_y:.2f}) - {corner_type}")
            else:
                rospy.logdebug(f"Blocked node rejected: ({global_x:.2f}, {global_y:.2f}) - {corner_type}")
        
        rospy.loginfo(f"Line-of-sight validation: {valid_nodes_count}/{len(all_corners)} nodes are reachable")
        goal_reachable = self.has_clear_path(self.current_pose[0], self.current_pose[1], self.goal_x, self.goal_y)
        if goal_reachable:
            self.nodes.append((self.goal_x, self.goal_y, 'goal'))
            rospy.loginfo("Goal is directly reachable")
        else:
            rospy.logwarn("Goal is not directly reachable - will use intermediate waypoints")
            self.nodes.append((self.goal_x, self.goal_y, 'goal'))
        
        # checks if there is a viable node available --> if not, counter increments
        if valid_nodes_count == 0:
            self.rotation_count += 1
            rospy.logwarn(f"No viable nodes found! Rotation count: {self.rotation_count}/{self.max_rotations_before_edge_fallback}")
            if not self.edge_fallback_active:
                rospy.logwarn("Re-scanning for corners...")
                self.current_state = "ROTATING"
                self.start_rotation()
                return
            else:
                rospy.logwarn("Edge fallback mode: No viable edge nodes found! Re-scanning...")
                self.current_state = "ROTATING"
                self.start_rotation()
                return
        else:
            if self.rotation_count > 0:
                rospy.loginfo(f"Found viable nodes! Resetting rotation counter (was {self.rotation_count})")
                self.rotation_count = 0
        self.visualize_nodes()
        if len(self.nodes) > 1:
            next_node_idx = self.run_dijkstra()
            self.target_node = self.nodes[next_node_idx]
            self.scan_processed = True
            self.current_state = "MOVING"
            mode_info = "edge fallback" if self.edge_fallback_active else "corner detection"
            rospy.loginfo(f"Moving to node: ({self.target_node[0]:.2f}, {self.target_node[1]:.2f}) - Mode: {mode_info}")
        else:
            rospy.logwarn("No valid nodes found! Re-scanning...")
            self.current_state = "ROTATING"
            self.start_rotation()
    
    def run_dijkstra(self):
        """Run Dijkstra's algorithm to find closest node to goal."""
        goal = self.nodes[-1]
        goal_pos = (goal[0], goal[1])
        min_dist = float('inf') #infinite is chosen, because that is the traditional "default" when performing the algorithm, for positions we do not know
        next_node_idx = 0
        rospy.loginfo("\nNode Evaluation:")
        rospy.loginfo("Index | Coordinates       | Type       | Distance to Goal | Current to Node | Total Weight")
        rospy.loginfo("------|-------------------|------------|------------------|-----------------|-------------")
        for i, node in enumerate(self.nodes[:-1]):
            node_pos = (node[0], node[1])
            dist = np.linalg.norm(np.array(node_pos) - np.array(goal_pos))
            current_to_node = np.linalg.norm(np.array(node_pos) - np.array(self.current_pose[:2]))
            weight = dist * (1 + 0.2 * current_to_node / max(dist, 0.1))

            # edge_mult = 1.2
            # node_mult = 0.9
            # if node[2] == "edge":
            #     weight *= edge_mult
            # elif node[2] == "convex":
            #     weight *= node_mult

#NOTE: This weighting metric is very simple --> picked arbitrarily to punish nodes that are far from (self.goal_x, self.goal_y),
    #while minimizing current-coordinate to node-coordinate distance

#NOTE: As edge_mult approaches node_mult, edges are more likely to be as equally weighted as nodes
    #Thus, the pool of selectable nodes increases

#CAUTION: In general, one should be careful of edges, becuase they can often lead to undesireable collisions with the wall, causing infinite loops

            rospy.loginfo(f"{i:5d} | ({node[0]:.2f}, {node[1]:.2f}) | {node[2]:<10} | {dist:16.2f} | {current_to_node:15.2f} | {weight:12.2f}")

            if weight < min_dist:
                min_dist = weight
                next_node_idx = i
        
        rospy.loginfo(f"Selected target: Node {next_node_idx} at ({self.nodes[next_node_idx][0]:.2f}, {self.nodes[next_node_idx][1]:.2f}) with total weight {min_dist:.2f}")
        return next_node_idx
    
    def imu_callback(self, msg):
        #dummy callback to ensure IMU data is available for EnhancedBasePIDNode
        pass
    
    def move_to_node(self):
        """Move to target node using EnhancedBasePIDNode with external monitoring"""
        if self.target_node is None or self.current_pose is None:
            rospy.logwarn("No target node or current pose. Cannot move.")
            return
            
        target_x, target_y, node_type = self.target_node
        if self.current_state != "GOAL_IN_SIGHT":
            # Turn towards target node
            direction_to_node = math.atan2(
                target_y - self.current_pose[1],
                target_x - self.current_pose[0]
            )
            rospy.loginfo(f"Turning towards node at ({target_x:.2f}, {target_y:.2f})")
            try:
                turn_controller = EnhancedBasePIDNode(self.current_pose[0], self.current_pose[1], direction_to_node)
                self.current_pid_controller = turn_controller
                turn_thread = threading.Thread(target=turn_controller.run_controller)
                turn_thread.daemon = True
                turn_thread.start()
                self.monitor_pid_controller(turn_controller, "turn")
                rospy.loginfo("Turn towards node complete. Now moving...")
            except Exception as e:
                rospy.logerr(f"Turn towards node failed: {str(e)}")
        if node_type == 'goal':
            target_yaw = self.start_yaw
        else:
            goal_node = self.nodes[-1]
            target_yaw = math.atan2(
                goal_node[1] - target_y,
                goal_node[0] - target_x
            )

        rospy.loginfo(f"Moving to target at ({target_x:.2f}, {target_y:.2f})")
        try:
            pid_controller = EnhancedBasePIDNode(target_x, target_y, target_yaw)
            self.current_pid_controller = pid_controller
            move_thread = threading.Thread(target=pid_controller.run_controller)
            move_thread.daemon = True
            move_thread.start()
            self.monitor_pid_controller(pid_controller, "move")
        except Exception as e:
            rospy.logerr(f"Movement failed: {str(e)}")
        finally:
            self.current_pid_controller = None
            if node_type == 'goal':
                rospy.loginfo("Reached goal! Navigation complete.")
                self.save_all_data()
                rospy.signal_shutdown("Goal reached")

            else:
                rospy.loginfo("Node approach complete. Starting next scan...")
                self.current_state = "ROTATING"
                self.start_rotation()
    
    def shutdown_handler(self):
        """Handle shutdown and save data"""
        rospy.loginfo("Shutdown requested - saving data...")
        self.save_all_data()   

    def monitor_pid_controller(self, controller, phase):
        """Monitor PID controller and stop it when error thresholds are met"""
        start_time = rospy.Time.now()
        consecutive_good_readings = 0
        required_consecutive_readings = 5
        
        rospy.loginfo(f"Starting {phase} monitoring - Position threshold: {self.position_error_threshold}m, "
                     f"Yaw threshold: {self.yaw_error_threshold}rad")
        
        while not rospy.is_shutdown() and controller.is_running:
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            if elapsed_time > self.max_movement_time:
                rospy.logwarn(f"{phase.capitalize()} timeout after {elapsed_time:.1f}s - stopping controller")
                controller.stop_controller()
                break
            try:
                position_error, yaw_error = controller.get_current_error()
                position_ok = position_error <= self.position_error_threshold
                yaw_ok = yaw_error <= self.yaw_error_threshold
                
                if position_ok and yaw_ok:
                    consecutive_good_readings += 1
                    rospy.logdebug(f"{phase.capitalize()} - Good reading {consecutive_good_readings}/{required_consecutive_readings}: "
                                 f"pos_err={position_error:.3f}m, yaw_err={yaw_error:.3f}rad")
                    
                    if consecutive_good_readings >= required_consecutive_readings:
                        rospy.loginfo(f"{phase.capitalize()} target reached! Final errors - "
                                    f"Position: {position_error:.3f}m, Yaw: {yaw_error:.3f}rad, "
                                    f"Time: {elapsed_time:.1f}s")
                        controller.stop_controller()
                        break
                else:
                    consecutive_good_readings = 0
                    rospy.logdebug(f"{phase.capitalize()} - Error too high: "
                                 f"pos_err={position_error:.3f}m ({'OK' if position_ok else 'HIGH'}), "
                                 f"yaw_err={yaw_error:.3f}rad ({'OK' if yaw_ok else 'HIGH'})")
            except Exception as e:
                rospy.logwarn(f"Error monitoring {phase} controller: {str(e)}")
                break
            self.error_check_rate.sleep()
        if controller.is_running:
            controller.stop_controller()
        rospy.loginfo(f"{phase.capitalize()} monitoring complete")
    def collect_data_point(self, velocity_cmd=None):
        if not self.data_collection_enabled or self.current_pose is None or self.start_time is None:
            return
        current_time = rospy.Time.now().to_sec()
        relative_time = current_time - self.start_time
        if self.last_position is not None:
            distance_increment = math.sqrt(
                (self.current_pose[0] - self.last_position[0])**2 + 
                (self.current_pose[1] - self.last_position[1])**2
            )
            self.total_distance_traveled += distance_increment
        starting_displacement = math.sqrt(
            (self.current_pose[0] - self.starting_position[0])**2 + 
            (self.current_pose[1] - self.starting_position[1])**2
        ) if self.starting_position else 0.0
        vel_x = velocity_cmd.linear.x if velocity_cmd else 0.0
        vel_y = velocity_cmd.linear.y if velocity_cmd else 0.0
        vel_yaw = velocity_cmd.angular.z if velocity_cmd else 0.0
        x_error = y_error = yaw_error = 0.0
        if self.target_node:
            x_error = self.target_node[0] - self.current_pose[0]
            y_error = self.target_node[1] - self.current_pose[1]
            target_yaw = math.atan2(y_error, x_error)
            raw_yaw_error = target_yaw - self.current_pose[2]
            yaw_error = math.atan2(math.sin(raw_yaw_error), math.cos(raw_yaw_error))
        data_point = {
            'time': relative_time,
            'x_pos': self.current_pose[0],
            'y_pos': self.current_pose[1],
            'yaw_pos': self.current_pose[2],
            'velocity_x': vel_x,
            'velocity_y': vel_y,
            'velocity_yaw': vel_yaw,
            'starting_displacement': starting_displacement,
            'total_distance_traveled': self.total_distance_traveled,
            'x_error': x_error,
            'y_error': y_error,
            'yaw_error': yaw_error
        }
        self.data_log.append(data_point)
        self.last_position = self.current_pose
    
    def save_data_to_csv(self):
        """Save collected data to CSV file"""
        if not self.data_log:
            rospy.logwarn("No data to save")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.world_name}_enhanced_pid_data_{timestamp}.csv"
        filepath = os.path.join(self.data_directory, filename)
        
        fieldnames = ['time', 'x_pos', 'y_pos', 'yaw_pos', 'velocity_x', 'velocity_y', 
                    'velocity_yaw', 'starting_displacement', 'total_distance_traveled', 
                    'x_error', 'y_error', 'yaw_error']
        
        with open(filepath, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.data_log)
        
        rospy.loginfo(f"Data saved to: {filepath}")

    def create_position_plots(self):
        """Create 4x4 subplot of position and orientation data"""
        if not self.data_log:
            return
        
        times = [d['time'] for d in self.data_log]
        x_pos = [d['x_pos'] for d in self.data_log]
        y_pos = [d['y_pos'] for d in self.data_log]
        yaw_pos = [d['yaw_pos'] for d in self.data_log]
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        ax1.plot(x_pos, y_pos, 'b-', linewidth=2)
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_title('Robot Trajectory (Y vs X)')
        ax1.grid(True)
        ax1.axis('equal')
        ax2.plot(times, x_pos, 'r-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('X Position (m)')
        ax2.set_title('X Position vs Time')
        ax2.grid(True)

        ax3.plot(times, y_pos, 'g-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Y Position (m)')
        ax3.set_title('Y Position vs Time')
        ax3.grid(True)

        ax4.plot(times, yaw_pos, 'm-', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Yaw (rad)')
        ax4.set_title('Yaw vs Time')
        ax4.grid(True)
        
        plt.tight_layout()
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.world_name}_enhanced_pid_positions_{timestamp}.png"
        filepath = os.path.join(self.data_directory, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        plt.close()
        
        rospy.loginfo(f"Position plots saved to: {filepath}")

    def create_error_plots(self):
        """Create overlapping error plots"""
        if not self.data_log:
            return
        
        times = [d['time'] for d in self.data_log]
        x_errors = [d['x_error'] for d in self.data_log]
        y_errors = [d['y_error'] for d in self.data_log]
        yaw_errors = [d['yaw_error'] for d in self.data_log]
        
        plt.figure(figsize=(12, 8))
        plt.plot(times, x_errors, 'r-', linewidth=2, label='X Error', alpha=0.8)
        plt.plot(times, y_errors, 'g-', linewidth=2, label='Y Error', alpha=0.8)
        plt.plot(times, yaw_errors, 'b-', linewidth=2, label='Yaw Error', alpha=0.8)
        
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        plt.title('Control Errors vs Time (Enhanced PID)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.world_name}_enhanced_pid_errors_{timestamp}.png"
        filepath = os.path.join(self.data_directory, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        plt.close()
        
        rospy.loginfo(f"Error plots saved to: {filepath}")

    def save_all_data(self):
        """Save all data and create all plots"""
        if self.data_collection_enabled:
            self.save_data_to_csv()
            self.create_position_plots()
            self.create_error_plots()
            rospy.loginfo("All data collection and visualization complete")

    def main_loop(self):
        """Main control loop running at 10Hz"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_state not in ["ROTATING", "PROCESSING_SCAN", "GOAL_IN_SIGHT"]:
                if self.has_clear_path(self.current_pose[0], self.current_pose[1], self.goal_x, self.goal_y):
                    rospy.loginfo("Goal is now in sight! Moving directly to goal.")
                    self.target_node = (self.goal_x, self.goal_y, 'goal')
                    self.current_state = "GOAL_IN_SIGHT"
            
            if self.current_state == "MOVING":
                self.move_to_node()
            elif self.current_state == "GOAL_IN_SIGHT":
                self.move_to_node()
            elif self.current_state == "ROTATING":
                pass
            elif self.current_state == "PROCESSING_SCAN":
                pass
            
            rate.sleep()
    
    def visualize_nodes(self):
        """Visualize graph nodes in RViz with color coding"""
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = "odom"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        for i, (x, y, corner_type) in enumerate(self.nodes[:-1]):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.SPHERE
            marker.id = i
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.lifetime = rospy.Duration(0)
            
            if corner_type == 'convex':
                marker.color = ColorRGBA(0, 1, 0, 1)  # convex corners are green
            elif corner_type == 'concave':
                marker.color = ColorRGBA(1, 1, 0, 1)  # concave corners are yellow
            elif corner_type == 'edge':
                marker.color = ColorRGBA(0, 1, 1, 1)  # edges are cyan
            marker.pose.position = Point(x, y, 0)
            marker_array.markers.append(marker)
        
        goal_node = self.nodes[-1]
        goal_marker = Marker()
        goal_marker.header.frame_id = "odom"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.type = Marker.SPHERE
        goal_marker.id = 100
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.3
        goal_marker.color = ColorRGBA(1, 0, 0, 1) # red for global marker (ultimate end-goal-node)
        goal_marker.pose.position = Point(goal_node[0], goal_node[1], 0)
        marker_array.markers.append(goal_marker)

        if self.target_node:
            target_marker = Marker()
            target_marker.header.frame_id = "odom"
            target_marker.header.stamp = rospy.Time.now()
            target_marker.type = Marker.SPHERE
            target_marker.id = 200
            target_marker.scale.x = target_marker.scale.y = target_marker.scale.z = 0.25
            target_marker.color = ColorRGBA(0, 0, 1, 1)  # blue for current target marker
            target_marker.pose.position = Point(self.target_node[0], self.target_node[1], 0)
            marker_array.markers.append(target_marker)
        
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        navigator = Navigator()
        navigator.main_loop()
    except rospy.ROSInterruptException:
        pass