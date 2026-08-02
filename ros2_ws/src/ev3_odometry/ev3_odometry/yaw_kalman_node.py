#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_from_quaternion(q) -> float:
    """Extract yaw (rotation about z) from a quaternion. Assumes planar motion (roll=pitch=0)."""
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class YawKalmanNode(Node):
    """Fuses encoder-based yaw (/odom) with gyro rate (/imu/raw) using a
    scalar 1D Kalman filter. Publishes three comparable signals:
    /yaw/encoder (raw, unfiltered), /yaw/imu (pure gyro integration,
    drifts, comparison only), /yaw/kalman (fused estimate).
    """

    def __init__(self):
        super().__init__('yaw_kalman_node')

        # Q: process noise (gyro model uncertainty), R: measurement noise
        # (encoder-based yaw uncertainty). Both are placeholders --
        # TODO: determine experimentally (Q from stationary IMU noise,
        # R from repeated odometry runs) once the IMU is mounted.
        self.declare_parameter('Q', 0.001)
        self.declare_parameter('R', 0.05)
        self.declare_parameter('initial_P', 1.0)

        self.Q = self.get_parameter('Q').value
        self.R = self.get_parameter('R').value

        # Kalman state
        self.theta_hat = 0.0
        self.P = self.get_parameter('initial_P').value
        self.last_imu_time = None

        # Pure gyro integration, for comparison only (not part of the filter)
        self.theta_imu = 0.0

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu/raw', self.imu_callback, 10)

        self.pub_enc = self.create_publisher(Float32, '/yaw/encoder', 10)
        self.pub_imu = self.create_publisher(Float32, '/yaw/imu', 10)
        self.pub_kf = self.create_publisher(Float32, '/yaw/kalman', 10)

        self.get_logger().info(
            f'yaw_kalman_node started. Q={self.Q}, R={self.R} '
            f'(both placeholders, TODO: determine experimentally)'
        )

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now()
        omega_z = msg.angular_velocity.z

        if self.last_imu_time is None:
            self.last_imu_time = now
            return

        dt = (now - self.last_imu_time).nanoseconds * 1e-9
        self.last_imu_time = now
        if dt <= 0.0:
            return

        # Prediction step
        self.theta_hat = wrap_to_pi(self.theta_hat + omega_z * dt)
        self.P += self.Q

        # Pure integration for comparison (drifts, not corrected)
        self.theta_imu = wrap_to_pi(self.theta_imu + omega_z * dt)

        self.pub_imu.publish(Float32(data=self.theta_imu))
        self.pub_kf.publish(Float32(data=self.theta_hat))

    def odom_callback(self, msg: Odometry):
        theta_enc = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pub_enc.publish(Float32(data=theta_enc))

        # Correction step
        innovation = wrap_to_pi(theta_enc - self.theta_hat)
        K = self.P / (self.P + self.R)
        self.theta_hat = wrap_to_pi(self.theta_hat + K * innovation)
        self.P = (1.0 - K) * self.P

        self.pub_kf.publish(Float32(data=self.theta_hat))


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
