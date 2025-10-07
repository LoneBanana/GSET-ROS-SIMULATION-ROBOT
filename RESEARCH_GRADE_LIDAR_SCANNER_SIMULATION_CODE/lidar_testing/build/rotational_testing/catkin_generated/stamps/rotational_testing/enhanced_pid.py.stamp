#!/usr/bin/env python3
import rospy
import math
import time
from collections import deque
from nav_msgs.m  
class EnhancedBasePIDNode:
    def __init__(self, x_target, y_target, yaw_target):
        # Tuned default gains for Burger
        self.x_target = x_target
        self.y_target = y_target
        self.yaw_target = yaw_target
        
        # Default PID parameters
        pd = [0.8, 0.01, 0.15, 0.26]  # distance: [Kp, Ki, Kd, max_out]
        py = [1.5, 0.005, 0.1, 1.5]   # yaw: [Kp, Ki, Kd, max_out]_init__(self, x_target, y_target, yaw_target):
        # Tuned default gains for Burger
        pd = [0.8, 0.01, 0.15, 0.26]  # Default PID parameters for distance
        py = [1.5, 0.005, 0.1, 1.5]   # Default PID parameters for yawort Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion


class EnhancedPID:
    def __init__(self, kp, ki, kd, max_out, integ_window=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral_queue = deque(maxlen=integ_window)
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral_queue.clear()
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error):
        now = rospy.Time.now().to_sec()
        dt = now - self.prev_time if self.prev_time is not None else 0.1
        self.prev_time = now

        # Clamp error to prevent windup during large deviations
        clamped_error = max(-1.0, min(1.0, error))
        P = self.kp * clamped_error

        # Integral term with conditional accumulation
        if abs(clamped_error) < 0.5:  # Only accumulate when close to target
            self.integral_queue.append(clamped_error * dt)
        I = self.ki * sum(self.integral_queue)

        # Smoothed derivative term
        D = self.kd * (clamped_error - self.prev_error) / dt if dt > 0 else 0.0

        u = P + I + D
        u_sat = max(-self.max_out, min(self.max_out, u))

        # Conditional anti-windup - only pop if queue has items
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
        max_kp = 5.0  # Safer upper limit for Burger

        while kp < max_kp and oscillation_count < 6:
            error = self.get_control_signal()
            errors.append((time.time(), error))

            if len(errors) > 2:
                _, e1 = errors[-3]
                _, e2 = errors[-2]
                _, e3 = errors[-1]
                # Improved oscillation detection with amplitude threshold
                if ((e1 < e2 > e3 and abs(e2) > 0.05) or 
                        (e1 > e2 < e3 and abs(e2) > 0.05)):
                    oscillation_count += 1
                    last_times.append(time.time())

            kp *= 1.2  # More conservative increase
            self.set_gain(kp, 0.0, 0.0)
            rospy.sleep(1.0 / self.sample_rate)

        if oscillation_count >= 4 and len(last_times) > 2:
            periods = [last_times[i] - last_times[i-1] 
                      for i in range(1, len(last_times))]
            Tu = sum(periods) / len(periods)
            Ku = kp / 1.2  # Compensate for last increment
            rospy.loginfo(f"Found Ku={Ku:.3f}, Tu={Tu:.3f}")
        else:
            rospy.logwarn("Tuning failed: Using conservative defaults")
            Ku = 1.5
            Tu = 2.0

        return Ku, Tu


class EnhancedBasePIDNode:
    def __init__(self, x_target, y_target, yaw_target, standalone=True):
        if standalone:
            rospy.init_node('enhanced_base_pid')
        
        # Tuned default gains for Burger
        pd = rospy.get_param('~pid_distance', [0.8, 0.01, 0.15, 0.26])  # Reduced max_out
        py = rospy.get_param('~pid_yaw', [1.5, 0.005, 0.1, 1.5])  # Reduced gains
        self.pid_dist = EnhancedPID(*pd)
        self.pid_yaw = EnhancedPID(*py)
        self.autotune = rospy.get_param('~autotune', False)
        self.x_target, self.y_target, self.yaw_target = x_target, y_target, yaw_target
        self.x_actual = self.y_actual = self.yaw_actual = 0.0
        
        # Motion constraints for Burger
        self.slow_speed_thresh = rospy.get_param('~slow_speed_thresh', 0.05)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.22)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.84)

        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/imu', Imu, self.imu_cb)
        rospy.Subscriber('/target_pose', PoseStamped, self.goal_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Initialize prev_time for PIDs
        self.pid_dist.prev_time = rospy.Time.now().to_sec()
        self.pid_yaw.prev_time = rospy.Time.now().to_sec()
        
        if self.autotune:
            self.run_tuner()
            
        self.rate = rospy.Rate(20)  # Reduced control frequency
        self.run_controller()

    def odom_cb(self, msg):
        self.x_actual = msg.pose.pose.position.x
        self.y_actual = msg.pose.pose.position.y

    def imu_cb(self, msg):
        q = msg.orientation
        _, _, self.yaw_actual = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def goal_cb(self, msg):
        self.x_target = msg.pose.position.x
        self.y_target = msg.pose.position.y
        _, _, self.yaw_target = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        self.pid_dist.reset()
        self.pid_yaw.reset()
        # Reinitialize prev_time after reset
        self.pid_dist.prev_time = rospy.Time.now().to_sec()
        self.pid_yaw.prev_time = rospy.Time.now().to_sec()
        rospy.loginfo(
            f"New target: x={self.x_target:.2f}, y={self.y_target:.2f}, "
            f"yaw={self.yaw_target:.2f}"
        )

    def run_tuner(self):
        def get_err():
            dx = self.x_target - self.x_actual
            dy = self.y_target - self.y_actual
            return dx * math.cos(self.yaw_actual) + dy * math.sin(self.yaw_actual)
        
        def set_gains(kp, ki, kd):
            self.pid_dist.kp, self.pid_dist.ki, self.pid_dist.kd = kp, ki, kd
            
        tuner = ZeiglerNicholsTuner(get_err, set_gains, self.x_target)
        Ku, Tu = tuner.find_ultimate_parameters()
        
        # Conservative Ziegler-Nichols tuning
        Kp = 0.4 * Ku  # Reduced from 0.6
        Ki = 0.8 * Ku / Tu if Tu > 0 else 0.0  # Reduced from 1.2
        Kd = 0.05 * Ku * Tu  # Reduced from 0.075
        rospy.loginfo(f"Tuned gains: Kp={Kp:.3f}, Ki={Ki:.3f}, Kd={Kd:.3f}")
        self.pid_dist.kp, self.pid_dist.ki, self.pid_dist.kd = Kp, Ki, Kd

    def run_controller(self):
        while not rospy.is_shutdown():
            dx = self.x_target - self.x_actual
            dy = self.y_target - self.y_actual
            distance = math.sqrt(dx**2 + dy**2)
            
            # Calculate errors
            forward_err = dx * math.cos(self.yaw_actual) + dy * math.sin(self.yaw_actual)
            heading = math.atan2(dy, dx)
            yaw_err_heading = math.atan2(
                math.sin(heading - self.yaw_actual),
                math.cos(heading - self.yaw_actual)
            )
            
            # Speed-adaptive control strategy
            v = self.pid_dist.update(forward_err)
            
            # Determine yaw error source based on distance
            if distance > 0.1:  # Far from target - follow path heading
                psi_err = yaw_err_heading
            else:  # Close to target - align with target orientation
                raw = self.yaw_target - self.yaw_actual
                psi_err = math.atan2(math.sin(raw), math.cos(raw))
                
            # Apply steering constraints
            omega = self.pid_yaw.update(psi_err)
            
            # Coupled motion constraints (Burger-specific)
            abs_v = abs(v)
            abs_omega = abs(omega)
            
            # Reduce angular velocity when moving slowly
            if abs_v < 0.05:
                omega *= 0.7
                
            # Limit turn radius to prevent spin-in-place
            if abs_v < 0.01 and abs_omega > 0.5:
                v = 0.01 * (1 if v >= 0 else -1)  # Minimal movement
                
            # Apply absolute limits
            v = max(-self.max_linear_speed, min(self.max_linear_speed, v))
            omega = max(-self.max_angular_speed, min(self.max_angular_speed, omega))
            
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = omega
            self.cmd_pub.publish(cmd)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        EnhancedBasePIDNode(0,0,0)
    except rospy.ROSInterruptException:
        pass