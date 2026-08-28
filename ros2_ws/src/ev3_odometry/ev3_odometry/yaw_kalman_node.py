#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu


def yaw_from_quaternion(q) -> float:
    """Extract yaw (rotation about z) from a quaternion. Assumes planar motion (roll=pitch=0)."""
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a 2D yaw angle to a quaternion (rotation about z only)."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class YawKalmanNode(Node):
    """Fuses encoder-based yaw (/odom) with gyro rate (/imu/raw) using a
    scalar 1D Kalman filter. Publishes three comparable signals:
    /yaw/encoder (raw, unfiltered), /yaw/imu (pure gyro integration,
    drifts, comparison only), /yaw/kalman (fused estimate).

    Also dead-reckons a fused (x, y) position from the filtered yaw and
    the last known velocity (from /odom twist), published as a separate
    nav_msgs/Odometry on /odom_filtered. Position is intentionally NOT
    part of the Kalman state itself -- same pattern as Goel/Roumeliotis
    (1999): filter only the linear quantity (yaw), integrate position
    downstream from the filtered result. Keeps the filter linear, no
    Jacobian needed.

    NOTE: this node does NOT broadcast a TF transform, to avoid
    conflicting with the odom->base_link transform already broadcast by
    ev3_odometry_node (two nodes broadcasting the same transform is a
    known ROS anti-pattern). /odom_filtered is data-only.
    """

    def __init__(self):
        super().__init__('yaw_kalman_node')

        # Q: process noise (gyro model uncertainty per predict step), R:
        # measurement noise (encoder-based yaw uncertainty).
        # Q: can now be estimated from stationary-phase gyro noise in your
        #    recorded bags -- see analyze_yaw_tests.py, which now also
        #    prints a suggested_Q value per run (Q ~= (sigma_omega * dt)^2).
        # R: still a placeholder -- proper value needs either (a) residual
        #    variance from a multi-point steering calibration fit, or
        #    (b) empirical variance from >=3 valid repeated test runs.
        #    TODO: replace once more circle-test runs are available.
        self.declare_parameter('Q', 0.001)
        self.declare_parameter('R', 0.05)
        self.declare_parameter('initial_P', 1.0)

        self.Q = self.get_parameter('Q').value
        self.R = self.get_parameter('R').value

        # Kalman state (yaw only)
        self.theta_hat = 0.0
        self.P = self.get_parameter('initial_P').value
        self.last_imu_time = None  # float seconds, derived from msg.header.stamp

        # Pure gyro integration, for comparison only (not part of the filter)
        self.theta_imu = 0.0

        # Downstream dead-reckoning state (x, y), fed by the fused theta_hat
        # and the last known velocity from /odom -- NOT part of the Kalman
        # filter state itself.
        self.x = 0.0
        self.y = 0.0
        self.last_v = 0.0

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu/raw', self.imu_callback, 10)

        self.pub_enc = self.create_publisher(Float32, '/yaw/encoder', 10)
        self.pub_imu = self.create_publisher(Float32, '/yaw/imu', 10)
        self.pub_kf = self.create_publisher(Float32, '/yaw/kalman', 10)
        self.pub_odom_filtered = self.create_publisher(Odometry, '/odom_filtered', 10)

        self.get_logger().info(
            f'yaw_kalman_node started. Q={self.Q}, R={self.R} '
            f'(Q: derive from bag data via analyze_yaw_tests.py, R: still a placeholder)'
        )

    def imu_callback(self, msg: Imu):
        omega_z = msg.angular_velocity.z

        # Use the STM32-embedded HAL_GetTick() timestamp (set by
        # imu_bridge_node in msg.header.stamp), NOT local ROS receive time.
        # This is the whole point of embedding the device timestamp: it
        # neutralizes TCP jitter. self.get_clock().now() would silently
        # reintroduce the exact problem this architecture decision solved.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_imu_time is None:
            self.last_imu_time = t
            return

        dt = t - self.last_imu_time
        self.last_imu_time = t
        if dt <= 0.0:
            # Non-monotonic timestamp (e.g. STM32 reboot resets HAL_GetTick()
            # back to 0) -- skip this step. Self-heals on the next callback
            # since last_imu_time is already updated above.
            return
        if dt > 0.5:
            # Unusually large gap (e.g. TCP reconnect) -- skip integrating
            # over it rather than injecting one large yaw jump.
            self.get_logger().warn(f'imu_callback: large dt={dt:.3f}s, skipping predict step')
            return

        # --- Predict step (yaw only) ---
        self.theta_hat = wrap_to_pi(self.theta_hat + omega_z * dt)
        self.P += self.Q

        # --- Downstream dead-reckoning (x, y) from fused theta_hat and the
        # last known velocity from /odom. Zero-order hold on v between
        # encoder updates (20 Hz) is fine since v changes slowly relative
        # to the 50 Hz predict rate. ---
        self.x += self.last_v * math.cos(self.theta_hat) * dt
        self.y += self.last_v * math.sin(self.theta_hat) * dt

        # Pure integration for comparison (drifts, not corrected)
        self.theta_imu = wrap_to_pi(self.theta_imu + omega_z * dt)

        self.pub_imu.publish(Float32(data=self.theta_imu))
        self.pub_kf.publish(Float32(data=self.theta_hat))
        self.publish_odom_filtered(msg.header.stamp)

    def odom_callback(self, msg: Odometry):
        theta_enc = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pub_enc.publish(Float32(data=theta_enc))
        self.last_v = msg.twist.twist.linear.x

        # --- Correction step (yaw only) ---
        innovation = wrap_to_pi(theta_enc - self.theta_hat)
        K = self.P / (self.P + self.R)
        self.theta_hat = wrap_to_pi(self.theta_hat + K * innovation)
        self.P = (1.0 - K) * self.P

        self.pub_kf.publish(Float32(data=self.theta_hat))

    def publish_odom_filtered(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta_hat)
        odom.twist.twist.linear.x = self.last_v
        self.pub_odom_filtered.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = YawKalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
