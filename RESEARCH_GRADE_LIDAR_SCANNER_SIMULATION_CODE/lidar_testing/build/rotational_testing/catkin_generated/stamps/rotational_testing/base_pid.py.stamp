#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

class PID:
    def __init__(self, kp, ki, kd, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out

        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None

    def update(self, error):
        """Compute PID output given current error."""
        now = rospy.Time.now().to_sec()
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = now - self.prev_time

        # Proportional term
        P = self.kp * error

        # Integral term
        if dt > 0:
            self.integral += error * dt
        I = self.ki * self.integral

        # Derivative term
        if self.prev_error is None or dt == 0:
            D = 0.0
        else:
            D = self.kd * (error - self.prev_error) / dt

        # Save for next iteration
        self.prev_time = now
        self.prev_error = error

        # Compute raw output and clamp
        u = P + I + D
        u = max(-self.max_out, min(self.max_out, u))
        return u

class BasePIDNode:
    def __init__(self):
        rospy.init_node('base_pid', anonymous=True)

        # Load PID gains and max outputs
        pd = rospy.get_param('~pid_distance', [1.0, 0.0, 0.2, 0.5])
        py = rospy.get_param('~pid_yaw',      [2.0, 0.0, 0.1, 1.0])

        self.pid_dist = PID(*pd)
        self.pid_yaw  = PID(*py)

        # State
        self.x_actual = 0.0
        self.y_actual = 0.0
        self.yaw_actual = 0.0

        self.x_target = 1.0
        self.y_target = 0.0
        self.yaw_target = 0.0

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/imu',  Imu,      self.imu_cb)
        rospy.Subscriber('/target_pose', PoseStamped, self.goal_cb)

        # Publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Control loop
        self.control_rate = rospy.Rate(10)  # 10 Hz
        self.run()

    def odom_cb(self, msg: Odometry):
        self.x_actual = msg.pose.pose.position.x
        self.y_actual = msg.pose.pose.position.y

    def imu_cb(self, msg: Imu):
        # Extract yaw from quaternion
        q = msg.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_actual = yaw

    def goal_cb(self, msg: PoseStamped):
        self.x_target   = msg.pose.position.x
        self.y_target   = msg.pose.position.y
        # If the goal includes orientation:
        q = msg.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_target = yaw
        # Optionally reset integrators when goal changes
        self.pid_dist.reset()
        self.pid_yaw.reset()

    def run(self):
        while not rospy.is_shutdown():
            # Compute planar distance error
            dx = self.x_target - self.x_actual
            dy = self.y_target - self.y_actual
            dist_err = math.hypot(dx, dy)

            # Compute desired heading to target
            desired_heading = math.atan2(dy, dx)
            # Yaw error: wrap to [-pi, pi]
            yaw_err = desired_heading - self.yaw_actual
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

            # PID outputs
            v_forward = self.pid_dist.update(dist_err)
            omega     = self.pid_yaw.update(yaw_err)

            # If you also want to track a yaw_target separate from heading-to-goal,
            # uncomment below and override omega:
            # yaw_direct_err = self.yaw_target - self.yaw_actual
            # yaw_direct_err = math.atan2(math.sin(yaw_direct_err), math.cos(yaw_direct_err))
            # omega = self.pid_yaw.update(yaw_direct_err)

            # Publish velocity
            cmd = Twist()
            cmd.linear.x  = v_forward
            cmd.angular.z = omega
            self.cmd_pub.publish(cmd)

            self.control_rate.sleep()

if __name__ == '__main__':
    try:
        BasePIDNode()
    except rospy.ROSInterruptException:
        pass