#!/usr/bin/env python3

#shebang strikes again!

import math
import os
import csv
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from datetime import datetime
import numpy as np
import rospy
from scipy.signal import medfilt, find_peaks
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

def save_csv_data(data_dict, controller_type, world_name):
    """
    save to csv:
        data_dict: dict --> keys: 
            "time", "x_pose", "y_pos", "yaw_pos", "velocity_x", "velocity_y", "velocity_yaw", "starting_displacement", "total_distance_traveled", "x_error", "y_error", "yaw_error"
        
        controller_type: str --> "P" or "Enhanced_PID" controllers are used
        
        world_name: str --> world file name (in this case, "house")
    """

    data_dir = "/home/lonebanana/lidar_testing/src/research_storage_data"
    os.makedirs(data_dir, exist_ok=True) #in case data_dir directory doesn't exist
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{controller_type}_{world_name}_data_{timestamp}.csv"
    filepath = os.path.join(data_dir, filename)
    headers = [
        "time", "x_pos", "y_pos", "yaw_pos", 
        "velocity_x", "velocity_y", "velocity_yaw",
        "starting_displacement", "total_distance_traveled",
        "x_error", "y_error", "yaw_error"
    ]

    try:
        with open(filepath, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(headers)
            data_length = len(data_dict["time"])
            for i in range(data_length):
                row = []
                for header in headers:
                    if header in data_dict:
                        row.append(data_dict[header][i])
                    else:
                        row.append("")  # Empty if data not available
                writer.writerow(row)
        print(f"CSV data saved successfully to: {filepath}")
        return filepath
    except Exception as e:
        print(f"Error saving CSV data: {e}")
        return None

def create_position_plots(data_dict, controller_type, world_name):
    """
    Create a 2x2 matplotlib plot showing position and orientation data.
    
    Args:
        data_dict (dict): Dictionary containing data lists
        controller_type (str): Type of controller ("P" or "Enhanced_PID")
        world_name (str): Name of the world file being tested
    """
    # Create directory if it doesn't exist
    data_dir = "/home/lonebanana/lidar_testing/src/research_storage_data"
    os.makedirs(data_dir, exist_ok=True)
    
    # Generate timestamp for unique filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{controller_type}_{world_name}_position_plots_{timestamp}.png"
    filepath = os.path.join(data_dir, filename)
    
    try:
        # Create 2x2 subplot
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle(f"{controller_type} Controller - {world_name} World\nPosition and Orientation Analysis", 
                     fontsize=14, fontweight="bold")
        
        # Plot 1: y_pos vs x_pos (trajectory)
        axes[0, 0].plot(data_dict["x_pos"], data_dict["y_pos"], "b-", linewidth=2, alpha=0.7)
        axes[0, 0].scatter(data_dict["x_pos"][0], data_dict["y_pos"][0], 
                          color="green", s=100, marker="o", label="Start", zorder=5)
        axes[0, 0].scatter(data_dict["x_pos"][-1], data_dict["y_pos"][-1], 
                          color="red", s=100, marker="s", label="End", zorder=5)
        axes[0, 0].set_xlabel("X Position (m)")
        axes[0, 0].set_ylabel("Y Position (m)")
        axes[0, 0].set_title("Robot Trajectory (Y vs X)")
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()
        axes[0, 0].set_aspect("equal", adjustable="box")
        
        # Plot 2: x_pos vs time
        axes[0, 1].plot(data_dict["time"], data_dict["x_pos"], "r-", linewidth=2)
        axes[0, 1].set_xlabel("Time (s)")
        axes[0, 1].set_ylabel("X Position (m)")
        axes[0, 1].set_title("X Position vs Time")
        axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: y_pos vs time
        axes[1, 0].plot(data_dict["time"], data_dict["y_pos"], "g-", linewidth=2)
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].set_ylabel("Y Position (m)")
        axes[1, 0].set_title("Y Position vs Time")
        axes[1, 0].grid(True, alpha=0.3)
        
        # Plot 4: yaw_pos vs time
        yaw_rad = [yaw for yaw in data_dict["yaw_pos"]]
        axes[1, 1].plot(data_dict["time"], yaw_rad, "m-", linewidth=2)
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].set_ylabel("Yaw Position (radians)")
        axes[1, 1].set_title("Yaw Position vs Time")
        axes[1, 1].grid(True, alpha=0.3)
        
        # Adjust layout to prevent overlap
        plt.tight_layout()
        
        # Save the plot
        plt.savefig(filepath, dpi=300, bbox_inches="tight")
        plt.close()  # Close to free memory
        
        print(f"Position plots saved successfully to: {filepath}")
        return filepath
        
    except Exception as e:
        print(f"Error creating position plots: {e}")
        return None

def create_error_plots(data_dict, controller_type, world_name):
    """
    Create overlapping plot of x_error, y_error, and yaw_error over time.
    
    Args:
        data_dict (dict): Dictionary containing data lists
        controller_type (str): Type of controller ("P" or "Enhanced_PID")
        world_name (str): Name of the world file being tested
    """
    # Create directory if it doesn't exist
    data_dir = "/home/lonebanana/lidar_testing/src/research_storage_data"
    os.makedirs(data_dir, exist_ok=True)
    
    # Generate timestamp for unique filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{controller_type}_{world_name}_error_plots_{timestamp}.png"
    filepath = os.path.join(data_dir, filename)
    
    try:
        fig, ax = plt.subplots(1, 1, figsize=(12, 8))
        ax.plot(data_dict["time"], data_dict["x_error"], "r-", linewidth=2.5, label="X Error", alpha=0.8)
        ax.plot(data_dict["time"], data_dict["y_error"], "b-", linewidth=2.5, label="Y Error", alpha=0.8)
        yaw_error_rad = [yaw_err for yaw_err in data_dict["yaw_error"]]
        ax.plot(data_dict["time"], yaw_error_rad, "g-", linewidth=2.5, label="Yaw Error (radians)", alpha=0.8)
        ax.set_xlabel("Time (s)", fontsize=12)
        ax.set_ylabel("Error", fontsize=12)
        ax.set_title(f"{controller_type} Controller - {world_name} World\nTracking Errors Over Time", fontsize=14, fontweight="bold")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=11, loc="best")
        ax.axhline(y=0, color="black", linestyle="--", alpha=0.5, linewidth=1) #zero-line
        #rms errors for x, y, and yaw in "error box" (top left)
        x_rms = np.sqrt(np.mean([x**2 for x in data_dict["x_error"]]))
        y_rms = np.sqrt(np.mean([y**2 for y in data_dict["y_error"]]))
        yaw_rms = np.sqrt(np.mean([y**2 for y in yaw_error_rad]))
        stats_text = f"RMS Errors:\nX: {x_rms:.4f} m\nY: {y_rms:.4f} m\nYaw: {yaw_rms:.2f}"
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=10, verticalalignment="top", bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))
        plt.tight_layout()
        plt.savefig(filepath, dpi=300, bbox_inches="tight")
        plt.close()
        print(f"Error plots saved successfully to: {filepath}")
        return filepath 
    except Exception as e:
        print(f"Error creating error plots: {e}")
        return None

#movement of TurtleBot3 Burger
class Navigator:
    def __init__(self):
        rospy.init_node("dijkstra_navigator", anonymous=True)
        self.goal_x = rospy.get_param("~goal_x", 3) #x-coord of goal set to 3 meters from origin along x-axis
        self.goal_y = rospy.get_param("~goal_y", 3) #y-coord of goal set to 3 meters from origin along y-axis
        self.rotation_speed = 0.3  # rad/s
        self.distance_threshold = 0.1 # meters --> distance threshold for moving to next node
        self.scan_processed = False
        self.is_rotating = False
        
        
        # information on the different states of the TurtleBot3 Burger
        #     "INIT", "ROTATING", "PROCESSING_SCAN", "MOVING"
        self.current_state = "INIT"

        #detection of corners
        self.min_corner_distance = 0    # meters --> depending on the configured goal_point, increasing may yield more optimal results
        self.max_corner_distance = 8    # meters --> range of LDS-02 LiDAR scanner --> 8 meters
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
        
        #test if point in space is "reachable" in straight line --> line-of-sight = los
        self.los_resolution = 0.1           # units between ray-casting-point (m)
        self.los_obstacle_threshold = 0.3   # min distance for obstacle clear path (m)

        # eyeshot --> when coordinate of (self.goal_x, self.goal_y) is w/i distance and reachable
        self.eyeshot_distance = 3.0

        self.robot_width = 0.178            # meters
        self.robot_safety_margin = 0.05
        self.robot_radius = (self.robot_width / 2) + self.robot_safety_margin

#NOTE: safety margin used for the robot can be optimized --> decreasing it will increase the point selection

        # edge-fallback mechanism --> after 10 rotations, if no nodes are found, will treat edges as nodes
        self.rotation_count = 0
        self.max_rotations_before_edge_fallback = 10
        self.edge_fallback_active = False # if True, will use edge detection fallback mechanism


        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.marker_pub = rospy.Publisher("/visualization_marker", MarkerArray, queue_size=10)

        # current state of robot
        self.current_pose = None
        self.start_pose = None
        self.start_yaw = None
        self.scan_pose = None
        self.corners = []
        self.edges = []
        self.nodes = []
        self.target_node = None
        self.latest_scan_ranges = None
        self.latest_scan_angles = None
        self.scan_angle_min = None
        self.scan_angle_max = None
        self.scan_angle_increment = None

        # dict --> figure-generation
        self.data_dict = {
            "time": [],
            "x_pos": [],
            "y_pos": [],
            "yaw_pos": [],
            "velocity_x": [],
            "velocity_y": [],
            "velocity_yaw": [],
            "starting_displacement": [],
            "total_distance_traveled": [],
            "x_error": [],
            "y_error": [],
            "yaw_error": []
        }
        self.start_time = rospy.Time.now().to_sec()
        self.prev_pos = None
        self.total_distance = 0

        rospy.loginfo("waiting for initial pose...")
        while not rospy.is_shutdown() and self.current_pose is None:
            rospy.sleep(0.1)

        self.start_pose = self.current_pose
        self.start_yaw = self.start_pose[2]
        rospy.loginfo(f"start pos: {self.start_pose}")

        # start 2*pi (rad) scan
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
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Starting 2*pi scan rotation...")

    def odom_callback(self, msg):
        #from robot current pose (/odom topic), corrects position based on offset
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        self.current_pose = (x, y, yaw)

        # storing collectred data
        current_time = rospy.Time.now().to_sec() - self.start_time
        self.data_dict["time"].append(current_time)
        self.data_dict["x_pos"].append(x)
        self.data_dict["y_pos"].append(y)
        self.data_dict["yaw_pos"].append(yaw)
        self.data_dict["velocity_x"].append(msg.twist.twist.linear.x)
        self.data_dict["velocity_y"].append(msg.twist.twist.linear.y)
        self.data_dict["velocity_yaw"].append(msg.twist.twist.angular.z)
        
        # traveled distance calculation
        if self.prev_pos is None:
            self.prev_pos = (x, y)
        else:
            dx = x - self.prev_pos[0]
            dy = y - self.prev_pos[1]
            self.total_distance += math.hypot(dx, dy)
            self.prev_pos = (x, y)
        
        if self.start_pose is not None:
            starting_displacement = math.hypot(x - self.start_pose[0], y - self.start_pose[1])
            self.data_dict["starting_displacement"].append(starting_displacement)
            self.data_dict["total_distance_traveled"].append(self.total_distance)
        else:
            self.data_dict["starting_displacement"].append(0.0)
            self.data_dict["total_distance_traveled"].append(0.0)
        if self.target_node:
            x_error = self.target_node[0] - x
            y_error = self.target_node[1] - y
            target_angle = math.atan2(y_error, x_error)
            yaw_error = target_angle - yaw
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
            self.data_dict["x_error"].append(x_error)
            self.data_dict["y_error"].append(y_error)
            self.data_dict["yaw_error"].append(yaw_error)
        else:
            self.data_dict["x_error"].append(0.0)
            self.data_dict["y_error"].append(0.0)
            self.data_dict["yaw_error"].append(0.0)
        if self.current_state == "ROTATING":
            duration = (rospy.Time.now() - self.rotation_start_time).to_sec()
            if duration >= (2 * np.pi / self.rotation_speed):
                self.cmd_vel_pub.publish(Twist())
                self.is_rotating = False
                self.current_state = "PROCESSING_SCAN"
                rospy.loginfo("2*pi scan complete. Ready to process scan.")
    
    def has_clear_path(self, start_x, start_y, end_x, end_y):
        #checks if two points in space have a clear path to one another
        if self.latest_scan_ranges is None or self.latest_scan_angles is None:
            rospy.logwarn("No scan data available for line-of-sight validation")
            return False
        dx = end_x - start_x
        dy = end_y - start_y
        path_distance = math.hypot(dx, dy)
        path_angle = math.atan2(dy, dx)
        perp_angle = path_angle + math.pi / 2
        num_checks = max(int(path_distance / self.los_resolution), 1)

        for i in range(1, num_checks + 1):
            t = i / num_checks
            cx = start_x + t * dx
            cy = start_y + t * dy

            for offset in [-self.robot_radius,
                           -self.robot_radius / 2,
                           0,
                           self.robot_radius / 2,
                           self.robot_radius]:
                px = cx + offset * math.cos(perp_angle)
                py = cy + offset * math.sin(perp_angle)
                # Transform to scan frame
                rel_x = px - self.scan_pose[0]
                rel_y = py - self.scan_pose[1]
                cos_yaw = math.cos(-self.scan_pose[2])
                sin_yaw = math.sin(-self.scan_pose[2])
                lx = rel_x * cos_yaw - rel_y * sin_yaw
                ly = rel_x * sin_yaw + rel_y * cos_yaw
                r = math.hypot(lx, ly)
                a = (math.atan2(ly, lx) + 2 * math.pi) % (2 * math.pi)
                idx = self.find_closest_ray_index(a)
                if idx is not None:
                    if self.latest_scan_ranges[idx] < r:
                        rospy.logdebug(
                            f"Path blocked at ({px:.2f}, {py:.2f}), offset={offset:.2f}"
                        )
                        return False
        return True

    def find_closest_ray_index(self, target_angle):
        #finds LiDAR ray closest to the desired angle
        if self.latest_scan_angles is None:
            return None
        diffs = np.abs(self.latest_scan_angles - target_angle)
        diffs = np.minimum(diffs, np.abs(diffs - 2 * np.pi))
        idx = int(np.argmin(diffs))
        r = self.latest_scan_ranges[idx]
        if np.isfinite(r) and r > 0:
            return idx
        return None

    def detect_edges(self, ranges, angles):
        #detects straight line segments in LiDAR data
        edges = []
        valid = np.where(
            (ranges > self.min_corner_distance) &
            (ranges < self.max_corner_distance)
        )[0]
        if len(valid) < 10:
            return edges
        vr = ranges[valid]
        va = angles[valid]
        if len(vr) > self.median_window:
            vr = medfilt(vr, kernel_size=self.median_window)
        xs = vr * np.cos(va)
        ys = vr * np.sin(va)
        window = 15
        i = 0
        while i < len(vr) - window:
            xw = xs[i:i + window]
            yw = ys[i:i + window]

            if len(xw) < 3:
                i += 1
                continue
            A = np.vstack([xw, np.ones(len(xw))]).T
            try:
                m, b = np.linalg.lstsq(A, yw, rcond=None)[0]
                line_y = m * xw + b
                dev = np.max(np.abs(yw - line_y))
                if dev < self.edge_deviation_threshold:
                    length = math.hypot(xw[-1] - xw[0], yw[-1] - yw[0])
                    if length > self.min_edge_length:
                        edges.append({
                            "start_range": vr[i],
                            "start_angle": va[i],
                            "end_range": vr[i + window - 1],
                            "end_angle": va[i + window - 1],
                            "length": length,
                            "slope": m,
                            "intercept": b
                        })
                        i += window // 2
                    else:
                        i += 1
                else:
                    i += 1
            except np.linalg.LinAlgError:
                i += 1
        return edges

    def detect_corners(self, ranges, angles):
        #corners found by finding peaks in the derivative of range
        valid = np.where(
            (ranges > self.min_corner_distance) &
            (ranges < self.max_corner_distance)
        )[0]
        vr = ranges[valid]
        va = angles[valid]
        if len(vr) > self.median_window:
            vr = medfilt(vr, kernel_size=self.median_window)
        grad = np.gradient(vr)
        peaks, _ = find_peaks(
            np.abs(grad),
            height=self.gradient_threshold,
            prominence=self.peak_prominence
        )
        corners = []
        for i in peaks:
            if 0 < i < len(vr) - 1:
                kind = "concave" if grad[i] < 0 else "convex"
                corners.append((vr[i], va[i], kind))

        rospy.loginfo(f"Derivative-based detection found {len(corners)} corners")
        return corners

    def convert_edges_to_corners(self, edges):
        #converts detected edges to nodes (when edges are absolutely necessary)
        pts = []
        for edge in edges:
            sx = edge["start_range"] * math.cos(edge["start_angle"])
            sy = edge["start_range"] * math.sin(edge["start_angle"])
            ex = edge["end_range"]   * math.cos(edge["end_angle"])
            ey = edge["end_range"]   * math.sin(edge["end_angle"])
            num = max(2, int(edge["length"] / self.edge_waypoint_spacing))
            for i in range(1, num):
                t = i / num
                x = sx + t * (ex - sx)
                y = sy + t * (ey - sy)
                r = math.hypot(x, y)
                a = math.atan2(y, x)
                pts.append((r, a, "edge"))
        return pts

    def scan_callback(self, scan_msg):
        #uses LiDAR for corner detection and data collection implementation
        # Always update latest scan data for line-of-sight checks
        self.latest_scan_ranges = np.array(scan_msg.ranges)
        self.latest_scan_angles = np.linspace(
            scan_msg.angle_min,
            scan_msg.angle_max,
            len(self.latest_scan_ranges)
        )
        self.scan_angle_min = scan_msg.angle_min
        self.scan_angle_max = scan_msg.angle_max
        self.scan_angle_increment = scan_msg.angle_increment

        if self.current_state != "PROCESSING_SCAN" or self.scan_processed:
            return
        rospy.loginfo("Processing scan data...")
        ranges = np.array(scan_msg.ranges)
        raw_angles = np.linspace(
            scan_msg.angle_min,
            scan_msg.angle_max,
            len(ranges)
        )
        if self.scan_pose is None:
            rospy.logwarn("No scan pose recorded! Must use current pose!")
            self.scan_pose = self.current_pose
        angles = (raw_angles + self.scan_pose[2]) % (2 * math.pi)
        self.corners = self.detect_corners(ranges, angles)
        rospy.loginfo(f"Detected {len(self.corners)} corners")
        # Edge fallback
        if not self.edge_fallback_active and self.rotation_count >= self.max_rotations_before_edge_fallback:
            rospy.logwarn("Activating edge fallback mode")
            self.edge_fallback_active = True
        edge_corners = []
        if self.edge_fallback_active:
            self.edges = self.detect_edges(ranges, raw_angles)
            rospy.loginfo(f"Edge fallback: Detected {len(self.edges)} edges")
            edge_corners = self.convert_edges_to_corners(self.edges)
            rospy.loginfo(f"Edge fallback: Generated {len(edge_corners)} edge-based corners")
        all_corners = self.corners + edge_corners
        # nodes made and chosen by validating with line-of-sight 
        self.nodes = []
        valid_count = 0
        for r, theta, ctype in all_corners:
            rr = r / 2.0
            gx = self.scan_pose[0] + rr * math.cos(theta)
            gy = self.scan_pose[1] + rr * math.sin(theta)

            if self.has_clear_path(self.scan_pose[0], self.scan_pose[1], gx, gy):
                self.nodes.append((gx, gy, ctype))
                valid_count += 1
        rospy.loginfo(f"{valid_count}/{len(all_corners)} nodes are reachable")
        goal_reachable = self.has_clear_path(
            self.scan_pose[0],
            self.scan_pose[1],
            self.goal_x,
            self.goal_y
        )
        if goal_reachable:
            rospy.loginfo("Goal is directly reachable")
        else:
            rospy.logwarn("Goal not directly reachable; will use intermediate waypoints")
        self.nodes.append((self.goal_x, self.goal_y, "goal"))

        if valid_count == 0:
            self.rotation_count += 1
            rospy.logwarn(f"No viable nodes! Rotation count: {self.rotation_count}")
            if not self.edge_fallback_active:
                rospy.logwarn("Re-scanning for corners...")
            else:
                rospy.logwarn("Edge fallback: no viable edge nodes; re-scanning...")
            self.current_state = "ROTATING"
            self.start_rotation()
            return

        if self.rotation_count > 0:
            rospy.loginfo(f"Resetting rotation counter (was {self.rotation_count})")
            self.rotation_count = 0

        self.visualize_nodes()

        if len(self.nodes) > 1:
            idx = self.run_dijkstra()
            if idx is not None:
                self.target_node = self.nodes[idx]
                self.scan_processed = True
                self.current_state = "MOVING"
                mode = "edge fallback" if self.edge_fallback_active else "corner detection"
                rospy.loginfo(f"Moving to node: {self.target_node[:2]} - Mode: {mode}")
            else:
                rospy.logwarn("No safely reachable nodes found. Rotating to scan from new angle...")
                self.current_state = "ROTATING"
                self.start_rotation()
        else:
            rospy.logwarn("No valid nodes found! Re-scanning...")
            self.current_state = "ROTATING"
            self.start_rotation()

    def run_dijkstra(self):
        # Dijkstra Algoritm to find best node
        goal = self.nodes[-1]
        goal_pos = np.array(goal[:2])
        current_pos = np.array(self.current_pose[:2])

        min_weight = float("inf")
        best_idx = None
        safety_multiplier = 2
        safety_margin = self.robot_radius * safety_multiplier
#NOTE: safety_multiplier may be optimized --> lowering will lead to more point selection
        rospy.loginfo("Evaluating nodes for Dijkstra:")
        for i, node in enumerate(self.nodes[:-1]):
            pos = np.array(node[:2])
            dist_current = np.linalg.norm(pos - current_pos)
            if dist_current < safety_margin:
                rospy.logdebug(f"Skipping node {i}: too close to current position")
                continue
            if not self.has_clear_path(
                self.current_pose[0], 
                self.current_pose[1], 
                node[0], 
                node[1]
            ):
                rospy.logdebug(f"Skipping node {i}: no clear path available")
                continue
            dist_goal = np.linalg.norm(pos - goal_pos)

            weight = dist_goal * (1 + 0.2 * dist_current / max(dist_goal, 0.1)) # punish nodes further from the goal, according to weighting metric

#NOTE: This weighting metric is very simple --> picked arbitrarily to punish nodes that are far from (self.goal_x, self.goal_y),
    #while minimizing current-coordinate to node-coordinate distance
            
            edge_mult = 1.2 # weight-multiplier: edge nodes preferred less
            node_mult = 0.9 # weight-multiplier: corners (convex) are preferred more

#NOTE: As edge_mult approaches node_mult, edges are more likely to be as equally weighted as nodes
    #Thus, the pool of selectable nodes increases
        
#CAUTION: In general, one should be careful of edges, becuase they can often lead to undesireable collisions with the wall, causing infinite loops

            if node[2] == "edge":
                weight *= edge_mult
            elif node[2] == "convex":
                weight *= node_mult   
            rospy.loginfo(f"Node {i}: {node[:2]}, type={node[2]}, dist_goal={dist_goal:.2f}, "
                         f"dist_current={dist_current:.2f}, weight={weight:.2f}")            
            if weight < min_weight:
                min_weight = weight
                best_idx = i
        if best_idx is not None:
            rospy.loginfo(f"Selected node {best_idx} with weight {min_weight:.2f}")
            return best_idx
        else:
            rospy.logwarn("No valid nodes found that are safely reachable")
            return None

    def move_to_node(self):
        #move to target node using P-controller
        if not self.target_node or not self.current_pose:
            rospy.logwarn("No target node or current pose available")
            return
        #check if goal is within eyeshot
        goal_node = self.nodes[-1]
        dx_goal = goal_node[0] - self.current_pose[0]
        dy_goal = goal_node[1] - self.current_pose[1]
        dist_to_goal = math.hypot(dx_goal, dy_goal)
        if (dist_to_goal < self.eyeshot_distance and 
            self.target_node != goal_node and
            self.has_clear_path(self.current_pose[0], self.current_pose[1], goal_node[0], goal_node[1])):
            rospy.loginfo(f"Goal within eyeshot ({dist_to_goal:.2f}m). Switching to direct navigation.")
            self.target_node = goal_node
        tx, ty, _ = self.target_node
        dx = tx - self.current_pose[0]
        dy = ty - self.current_pose[1]
        dist_err = math.hypot(dx, dy)
        if dist_err > self.distance_threshold:
            angle_err = math.atan2(dy, dx) - self.current_pose[2]
            angle_err = (angle_err + np.pi) % (2 * np.pi) - np.pi
            twist = Twist()
            twist.linear.x = min(0.5 * dist_err, 0.2)
            twist.angular.z = 1.0 * angle_err
            self.cmd_vel_pub.publish(twist)
        else:
            self.cmd_vel_pub.publish(Twist())
            if self.target_node[2] == "goal":
                rospy.loginfo("Reached the goal! Saving simulation data...")
                self.save_simulation_data()  # Add this line
                rospy.loginfo("Mission complete.")
                rospy.signal_shutdown("Goal reached")
            else:
                rospy.loginfo("Reached node. Starting next scan...")
                self.current_state = "ROTATING"
                self.start_rotation()

    def visualize_nodes(self):
        """Publish visualization markers for detected nodes."""
        ma = MarkerArray()
        # Clear old markers
        clear = Marker()
        clear.header.frame_id = "odom"
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i, (x, y, ctype) in enumerate(self.nodes[:-1]):
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = rospy.Time.now()
            m.type = Marker.SPHERE
            m.id = i
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.lifetime = rospy.Duration(0)
            if ctype == "convex":
                m.color = ColorRGBA(0, 1, 0, 1)
            elif ctype == "concave":
                m.color = ColorRGBA(1, 1, 0, 1)
            else:  # edge
                m.color = ColorRGBA(0, 1, 1, 1)
            m.pose.position = Point(x, y, 0)
            ma.markers.append(m)
        # register goal-nodes + marking visually (red)
        gx, gy, _ = self.nodes[-1]
        gm = Marker()
        gm.header.frame_id = "odom"
        gm.header.stamp = rospy.Time.now()
        gm.type = Marker.SPHERE
        gm.id = 100
        gm.scale.x = gm.scale.y = gm.scale.z = 0.3
        gm.color = ColorRGBA(1, 0, 0, 1)
        gm.pose.position = Point(gx, gy, 0)
        ma.markers.append(gm)
        # register target node + mark visually (blue)
        if self.target_node:
            tx, ty, _ = self.target_node
            tm = Marker()
            tm.header.frame_id = "odom"
            tm.header.stamp = rospy.Time.now()
            tm.type = Marker.SPHERE
            tm.id = 200
            tm.scale.x = tm.scale.y = tm.scale.z = 0.25
            tm.color = ColorRGBA(0, 0, 1, 1)
            tm.pose.position = Point(tx, ty, 0)
            ma.markers.append(tm)
        self.marker_pub.publish(ma)

    def save_simulation_data(self):
        controller_type = "P_Controller"
        world_name = "house"
        try:
            csv_path = save_csv_data(self.data_dict, controller_type, world_name)
            pos_plot_path = create_position_plots(self.data_dict, controller_type, world_name)
            err_plot_path = create_error_plots(self.data_dict, controller_type, world_name)  
            rospy.loginfo("Saved simulation data:")
            rospy.loginfo(f"CSV data: {csv_path}")
            rospy.loginfo(f"Position plots: {pos_plot_path}")
            rospy.loginfo(f"Error plots: {err_plot_path}")
        except Exception as e:
            rospy.logerr(f"Error saving simulation data: {e}")

    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_state == "MOVING":
                self.move_to_node()
            rate.sleep()


if __name__ == "__main__":
    try:
        navigator = Navigator()
        navigator.main_loop()
    except rospy.ROSInterruptException:
        pass