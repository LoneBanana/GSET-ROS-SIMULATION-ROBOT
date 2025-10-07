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

class Navigator:
    def __init__(self):
        rospy.init_node('dijkstra_navigator', anonymous=True)
        
        # Parameters
        self.goal_x = rospy.get_param('~goal_x', 3.0)
        self.goal_y = rospy.get_param('~goal_y', 3.0)
        self.rotation_speed = 0.3  # rad/s
        self.distance_threshold = 0.1
        self.scan_processed = False
        self.is_rotating = False
        self.current_state = "INIT"  # INIT, ROTATING, PROCESSING_SCAN, MOVING
        
        # Corner detection parameters
        self.min_corner_distance = 0  # meters (ignore closer points)
        self.max_corner_distance = 8   # meters (ignore farther points)
        self.peak_threshold = 0.2        # meters (minimum range difference for corner)
        self.window_size = 5              # smoothing window size
        self.median_window = 5
        
        # NEW: Derivative-based corner detection parameters
        self.gradient_threshold = 0.15  # minimum gradient magnitude for corner detection (meters/rad)
        self.peak_prominence = 0.1      # minimum prominence for gradient peaks
        
        # Edge detection parameters
        self.min_edge_length = 1.0       # minimum edge length to consider (meters)
        self.edge_deviation_threshold = 0.1  # max deviation from straight line (meters)
        self.edge_waypoint_spacing = 0.5 # spacing between waypoints on edges (meters)
        
        # Line-of-sight validation parameters
        self.los_resolution = 0.1  # meters between ray-casting points
        self.los_obstacle_threshold = 0.3  # minimum distance to obstacle for clear path
        
        # TurtleBot3 Burger physical dimensions
        self.robot_width = 0.178  # meters (TurtleBot3 Burger width)
        self.robot_safety_margin = 0.05  # additional safety margin
        self.robot_radius = (self.robot_width / 2) + self.robot_safety_margin  # effective radius for path planning
        
        # NEW: Edge fallback mechanism
        self.rotation_count = 0  # Count rotations without viable corners
        self.max_rotations_before_edge_fallback = 10  # Threshold for edge fallback
        self.edge_fallback_active = False  # Flag to indicate edge detection mode
        
        # Subscribers & Publishers
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', MarkerArray, queue_size=10)
        
        # Robot state
        self.current_pose = None
        self.start_pose = None
        self.corners = []
        self.edges = []  # New: store detected edges
        self.nodes = []
        self.target_node = None
        self.scan_angles = []
        self.scan_ranges = []
        # FIX: Store the pose where each scan was taken
        self.scan_pose = None
        
        # Store latest scan data for line-of-sight validation
        self.latest_scan_ranges = None
        self.latest_scan_angles = None
        self.scan_angle_min = None
        self.scan_angle_max = None
        self.scan_angle_increment = None
        
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
        """Start the 360-degree rotation for scanning"""
        self.is_rotating = True
        self.scan_processed = False
        self.rotation_start_time = rospy.Time.now()
        self.scan_angles = []
        self.scan_ranges = []
        # FIX: Record where the scan starts
        self.scan_pose = self.current_pose
        
        twist = Twist()
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Starting 360° scan rotation...")
    
    def odom_callback(self, msg):
        """Update current robot pose from odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (x, y, yaw)
        
        # Check if rotation should stop
        if self.current_state == "ROTATING":
            rotation_duration = (rospy.Time.now() - self.rotation_start_time).to_sec()
            if rotation_duration >= (2 * np.pi / self.rotation_speed):
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.is_rotating = False
                self.current_state = "PROCESSING_SCAN"
                rospy.loginfo("360° scan complete. Ready to process scan.")
    
    def has_clear_path(self, start_x, start_y, end_x, end_y):
        """Check if there's a clear path between two points using LiDAR data, considering robot width"""
        if self.latest_scan_ranges is None or self.latest_scan_angles is None:
            rospy.logwarn("No scan data available for line-of-sight validation")
            return False
        
        # Calculate distance and direction between start and end points
        path_distance = np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        path_angle = np.arctan2(end_y - start_y, end_x - start_x)
        
        # Calculate perpendicular direction for width checking
        perp_angle = path_angle + np.pi/2
        
        # Number of points to check along the path
        num_checks = max(int(path_distance / self.los_resolution), 1)
        
        # Check points along the path
        for i in range(1, num_checks + 1):  # Skip start point (i=0)
            t = i / num_checks
            center_x = start_x + t * (end_x - start_x)
            center_y = start_y + t * (end_y - start_y)
            
            # Check multiple points across the robot's width
            # Check center, left edge, right edge, and intermediate points
            width_check_points = [-self.robot_radius, -self.robot_radius/2, 0, 
                                self.robot_radius/2, self.robot_radius]
            
            for width_offset in width_check_points:
                # Calculate check point offset by robot width
                check_x = center_x + width_offset * np.cos(perp_angle)
                check_y = center_y + width_offset * np.sin(perp_angle)
                
                # Convert check point to polar coordinates relative to scan pose
                rel_x = check_x - self.scan_pose[0]
                rel_y = check_y - self.scan_pose[1]
                
                #Be careful of this: not entirely sure if this is correct --> saw this was one robust option obline
                cos_yaw = np.cos(-self.scan_pose[2])
                sin_yaw = np.sin(-self.scan_pose[2])
                local_x = rel_x * cos_yaw - rel_y * sin_yaw
                local_y = rel_x * sin_yaw + rel_y * cos_yaw
                
                check_range = np.sqrt(local_x**2 + local_y**2)
                check_angle = np.arctan2(local_y, local_x)
                
                # Normalize angle to [0, 2*pi] range
                check_angle = (check_angle + 2 * np.pi) % (2 * np.pi)
                
                # Find closest LiDAR ray to this angle
                closest_ray_idx = self.find_closest_ray_index(check_angle)
                
                if closest_ray_idx is not None:
                    measured_range = self.latest_scan_ranges[closest_ray_idx]
                    
                    # Check if this part of the robot would be blocked by an obstacle
                    # If measured range is less than check_range + safety margin, path is blocked
                    if measured_range < check_range:
                        rospy.logdebug(f"Path blocked at ({check_x:.2f}, {check_y:.2f}) "
                                     f"width_offset={width_offset:.2f}: "
                                     f"measured={measured_range:.2f}m, needed={check_range:.2f}m")
                        return False
        
        return True
    
    def find_closest_ray_index(self, target_angle):
        """Find the index of the LiDAR ray closest to the target angle"""
        if self.latest_scan_angles is None:
            return None
        
        # Find the closest angle index
        angle_diffs = np.abs(self.latest_scan_angles - target_angle)
        
        # Handle wrap-around case (e.g., target_angle near 0, ray near 2*pi)
        angle_diffs_wrapped = np.abs(self.latest_scan_angles - target_angle + 2*np.pi)
        angle_diffs_wrapped2 = np.abs(self.latest_scan_angles - target_angle - 2*np.pi)
        
        # Take minimum difference considering wrap-around
        angle_diffs = np.minimum(angle_diffs, np.minimum(angle_diffs_wrapped, angle_diffs_wrapped2))
        
        closest_idx = np.argmin(angle_diffs)
        
        # Verify the ray is valid (not nan/inf)
        if np.isfinite(self.latest_scan_ranges[closest_idx]) and self.latest_scan_ranges[closest_idx] > 0:
            return closest_idx
        
        return None
    
    def detect_edges(self, ranges, angles):
        """Detect straight line segments (edges) in LiDAR data"""
        edges = []
        
        # Filter valid data points
        valid_indices = np.where((ranges > self.min_corner_distance) & 
                                 (ranges < self.max_corner_distance))[0]
        if len(valid_indices) < 10:  # Need minimum points for edge detection
            return edges
            
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        if len(valid_ranges) > self.median_window:
            valid_ranges = medfilt(valid_ranges,kernel_size=self.median_window)

        # Convert to Cartesian coordinates for line fitting
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)
        
        # Use sliding window to detect line segments
        window_size = 15  # Number of points to consider for line fitting
        
        i = 0
        while i < len(valid_ranges) - window_size:
            # Extract window of points
            x_window = x_points[i:i+window_size]
            y_window = y_points[i:i+window_size]
            
            # Fit line using least squares
            if len(x_window) < 3:
                i += 1
                continue
                
            # Calculate line parameters (y = mx + b)
            try:
                A = np.vstack([x_window, np.ones(len(x_window))]).T
                m, b = np.linalg.lstsq(A, y_window, rcond=None)[0]
                
                # Calculate deviations from fitted line
                line_y = m * x_window + b
                deviations = np.abs(y_window - line_y)
                max_deviation = np.max(deviations)
                
                # Check if points form a good line
                if max_deviation < self.edge_deviation_threshold:
                    # Calculate edge length
                    edge_length = np.sqrt((x_window[-1] - x_window[0])**2 + 
                                        (y_window[-1] - y_window[0])**2)
                    
                    if edge_length > self.min_edge_length:
                        # Store edge as start and end points with angles
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
                        
                        # Skip ahead to avoid overlapping edges
                        i += window_size // 2
                    else:
                        i += 1
                else:
                    i += 1
                    
            except np.linalg.LinAlgError:
                i += 1
                
        return edges
    
    def detect_corners(self, ranges, angles):
        """Detect corners by finding local maxima/minima in range profile"""
        # Preprocess data
        valid_indices = np.where((ranges > self.min_corner_distance) & 
                                 (ranges < self.max_corner_distance))[0]
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Apply median filtering for noise reduction
        if len(valid_ranges) > self.median_window:
            valid_ranges = medfilt(valid_ranges, kernel_size=self.median_window)
        
        # NEW: Derivative-based corner detection
        # Compute gradient of range profile
        gradient = np.gradient(valid_ranges)
        abs_gradient = np.abs(gradient)
        
        # Find peaks in gradient magnitude
        peaks, _ = find_peaks(abs_gradient, 
                             height=self.gradient_threshold,
                             prominence=self.peak_prominence)
        
        corners = []
        for i in peaks:
            # Skip edge points
            if i == 0 or i == len(valid_ranges)-1:
                continue
                
            # Classify based on gradient sign
            if gradient[i] < 0:  # Negative gradient = concave corner
                corners.append((valid_ranges[i], valid_angles[i], 'concave'))
            else:  # Positive gradient = convex corner
                corners.append((valid_ranges[i], valid_angles[i], 'convex'))
        
        rospy.loginfo(f"Derivative-based detection found {len(corners)} corners")
        return corners
    
    def convert_edges_to_corners(self, edges):
        """Convert detected edges to corner-like waypoints for uniform processing"""
        edge_corners = []
        
        for edge in edges:
            # Generate waypoints along the edge, treating each as a "corner"
            # Calculate start and end points in Cartesian coordinates
            start_x = edge['start_range'] * np.cos(edge['start_angle'])
            start_y = edge['start_range'] * np.sin(edge['start_angle'])
            end_x = edge['end_range'] * np.cos(edge['end_angle'])
            end_y = edge['end_range'] * np.sin(edge['end_angle'])
            
            # Calculate number of waypoints based on edge length and spacing
            num_waypoints = max(2, int(edge['length'] / self.edge_waypoint_spacing))
            
            # Generate evenly spaced waypoints along the edge
            for i in range(1, num_waypoints):  # Skip start (0) and end (num_waypoints)
                t = i / num_waypoints  # Parameter from 0 to 1
                
                # Interpolate along the edge
                wp_x = start_x + t * (end_x - start_x)
                wp_y = start_y + t * (end_y - start_y)
                
                # Convert back to polar coordinates
                wp_range = np.sqrt(wp_x**2 + wp_y**2)
                wp_angle = np.arctan2(wp_y, wp_x)
                
                # Add as a "corner" with 'edge' type identifier
                edge_corners.append((wp_range, wp_angle, 'edge'))
                
        return edge_corners
    
    def scan_callback(self, scan_msg):
        """Process LiDAR data to detect corners and build navigation graph"""
        if self.current_state != "PROCESSING_SCAN" or self.scan_processed:
            return
            
        rospy.loginfo("Processing scan data...")
        
        # Convert LiDAR data to numpy arrays
        ranges = np.array(scan_msg.ranges)
        raw_angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Store scan data for line-of-sight validation
        self.latest_scan_ranges = ranges.copy()
        self.latest_scan_angles = raw_angles.copy()
        self.scan_angle_min = scan_msg.angle_min
        self.scan_angle_max = scan_msg.angle_max
        self.scan_angle_increment = scan_msg.angle_increment
        
        # FIX: Use the pose where scan was taken, not initial start pose
        if self.scan_pose is None:
            rospy.logwarn("No scan pose recorded! Using current pose.")
            self.scan_pose = self.current_pose
            
        # Transform angles to global frame using scan pose orientation
        angles = (raw_angles + self.scan_pose[2]) % (2 * np.pi)
        
        # Detect corners using derivative-based method
        self.corners = self.detect_corners(ranges, angles)
        rospy.loginfo(f"Detected {len(self.corners)} corners")
        
        # NEW: Check if edge fallback should be activated
        if not self.edge_fallback_active and self.rotation_count >= self.max_rotations_before_edge_fallback:
            rospy.logwarn(f"Activating edge fallback after {self.rotation_count} rotations without viable corners")
            self.edge_fallback_active = True
        
        # NEW: If edge fallback is active, detect edges and convert them to corners
        edge_corners = []
        if self.edge_fallback_active:
            self.edges = self.detect_edges(ranges, raw_angles)
            rospy.loginfo(f"Edge fallback mode: Detected {len(self.edges)} edges")
            
            # Convert edges to corner-like waypoints
            edge_corners = self.convert_edges_to_corners(self.edges)
            rospy.loginfo(f"Edge fallback mode: Generated {len(edge_corners)} edge-based corners")
            
            # Combine regular corners with edge-based corners
            all_corners = self.corners + edge_corners
        else:
            all_corners = self.corners
        
        # FIX: Generate graph nodes with correct coordinate transformation
        self.nodes = []
        valid_nodes_count = 0
        
        for r, theta, corner_type in all_corners:
            new_r = r / 2.0
            # Convert to global coordinates using scan pose as reference
            global_x = self.scan_pose[0] + new_r * np.cos(theta)
            global_y = self.scan_pose[1] + new_r * np.sin(theta)
            
            # Validate line-of-sight from current position to the potential node
            if self.has_clear_path(self.current_pose[0], self.current_pose[1], global_x, global_y):
                self.nodes.append((global_x, global_y, corner_type))
                valid_nodes_count += 1
                rospy.logdebug(f"Valid node: ({global_x:.2f}, {global_y:.2f}) - {corner_type}")
            else:
                rospy.logdebug(f"Blocked node rejected: ({global_x:.2f}, {global_y:.2f}) - {corner_type}")
        
        rospy.loginfo(f"Line-of-sight validation: {valid_nodes_count}/{len(all_corners)} nodes are reachable")
        
        # Add goal as final node (already in global coordinates)
        # Also validate line-of-sight to goal
        goal_reachable = self.has_clear_path(self.current_pose[0], self.current_pose[1], self.goal_x, self.goal_y)
        if goal_reachable:
            self.nodes.append((self.goal_x, self.goal_y, 'goal'))
            rospy.loginfo("Goal is directly reachable")
        else:
            rospy.logwarn("Goal is not directly reachable - will use intermediate waypoints")
            self.nodes.append((self.goal_x, self.goal_y, 'goal'))  # Add anyway for pathfinding
        
        # NEW: Check if we have viable nodes, increment rotation counter if not
        if valid_nodes_count == 0:
            self.rotation_count += 1
            rospy.logwarn(f"No viable nodes found! Rotation count: {self.rotation_count}/{self.max_rotations_before_edge_fallback}")
            
            # If we haven't reached the edge fallback threshold, try again
            if not self.edge_fallback_active:
                rospy.logwarn("Re-scanning for corners...")
                self.current_state = "ROTATING"
                self.start_rotation()
                return
            else:
                # Even in edge fallback mode, if no nodes found, continue trying
                rospy.logwarn("Edge fallback mode: No viable edge nodes found! Re-scanning...")
                self.current_state = "ROTATING"
                self.start_rotation()
                return
        else:
            # Reset rotation counter when viable nodes are found
            if self.rotation_count > 0:
                rospy.loginfo(f"Found viable nodes! Resetting rotation counter (was {self.rotation_count})")
                self.rotation_count = 0
        
        # Visualize nodes
        self.visualize_nodes()
        
        # Run Dijkstra and move to next node
        if len(self.nodes) > 1:  # Ensure we have at least 1 node + goal
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
        
        min_dist = float('inf')
        next_node_idx = 0
        
        rospy.loginfo("\nNode Evaluation:")
        rospy.loginfo("Index | Coordinates       | Type       | Distance to Goal | Current to Node | Total Weight")
        rospy.loginfo("------|-------------------|------------|------------------|-----------------|-------------")


        for i, node in enumerate(self.nodes[:-1]):
            node_pos = (node[0], node[1])
            dist = np.linalg.norm(np.array(node_pos) - np.array(goal_pos))
            
            # FIX: Calculate distance from current position for path length penalty
            current_to_node = np.linalg.norm(np.array(node_pos) - np.array(self.current_pose[:2]))
            weight = dist + 0.1 * current_to_node  # Penalize longer paths from current position

            rospy.loginfo(f"{i:5d} | ({node[0]:.2f}, {node[1]:.2f}) | {node[2]:<10} | {dist:16.2f} | {current_to_node:15.2f} | {weight:12.2f}")


            if weight < min_dist:
                min_dist = weight
                next_node_idx = i
        
        rospy.loginfo(f"Selected target: Node {next_node_idx} at ({self.nodes[next_node_idx][0]:.2f}, {self.nodes[next_node_idx][1]:.2f}) with total weight {min_dist:.2f}")
        return next_node_idx
    
    def move_to_node(self):
        """Move to target node using PID control"""
        if self.target_node is None or self.current_pose is None:
            rospy.logwarn("No target node or current pose. Cannot move.")
            return
            
        # FIX: Target node is now in global coordinates (x, y, type)
        target_x, target_y, _ = self.target_node
        
        # Calculate errors
        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        distance_error = math.sqrt(dx**2 + dy**2)
        
        # Only move if we're not already at the target
        if distance_error > self.distance_threshold:
            # PID parameters
            Kp_linear = 0.5
            Kp_angular = 1.0
            
            angle_error = math.atan2(dy, dx) - self.current_pose[2]
            angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
            
            # Generate control commands
            twist = Twist()
            twist.linear.x = min(Kp_linear * distance_error, 0.2)
            twist.angular.z = Kp_angular * angle_error
            self.cmd_vel_pub.publish(twist)
        else:
            # Reached the node
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo("Reached node. Starting next scan...")
            self.current_state = "ROTATING"
            self.start_rotation()
    
    def main_loop(self):
        """Main control loop running at 10Hz"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_state == "MOVING":
                self.move_to_node()
            rate.sleep()
    
    def visualize_nodes(self):
        """Visualize graph nodes in RViz with color coding"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header.frame_id = "odom"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # FIX: Node markers now use global coordinates directly
        for i, (x, y, corner_type) in enumerate(self.nodes[:-1]):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.SPHERE
            marker.id = i
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.lifetime = rospy.Duration(0)
            
            # Color coding based on corner type
            if corner_type == 'convex':
                marker.color = ColorRGBA(0, 1, 0, 1)  # Green
            elif corner_type == 'concave':
                marker.color = ColorRGBA(1, 1, 0, 1)  # Yellow
            elif corner_type == 'edge':  # NEW: Color for edge-based nodes
                marker.color = ColorRGBA(0, 1, 1, 1)  # Cyan
            
            # Use global coordinates directly
            marker.pose.position = Point(x, y, 0)
            marker_array.markers.append(marker)
        
        # Add goal marker (red) - already in global coordinates
        goal_node = self.nodes[-1]
        goal_marker = Marker()
        goal_marker.header.frame_id = "odom"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.type = Marker.SPHERE
        goal_marker.id = 100
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.3
        goal_marker.color = ColorRGBA(1, 0, 0, 1)  # Red
        goal_marker.pose.position = Point(goal_node[0], goal_node[1], 0)
        marker_array.markers.append(goal_marker)
        
        # Add current target marker (blue)
        if self.target_node:
            target_marker = Marker()
            target_marker.header.frame_id = "odom"
            target_marker.header.stamp = rospy.Time.now()
            target_marker.type = Marker.SPHERE
            target_marker.id = 200
            target_marker.scale.x = target_marker.scale.y = target_marker.scale.z = 0.25
            target_marker.color = ColorRGBA(0, 0, 1, 1)  # Blue
            target_marker.pose.position = Point(self.target_node[0], self.target_node[1], 0)
            marker_array.markers.append(target_marker)
        
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        navigator = Navigator()
        navigator.main_loop()
    except rospy.ROSInterruptException:
        pass