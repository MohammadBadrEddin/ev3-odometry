#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a 2D yaw angle to a quaternion (rotation about z only)."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class Ev3OdometryNode(Node):

    def __init__(self):
        super().__init__('ev3_odometry_node')

        # --- Parameters: physical constants ---
        self.declare_parameter('wheel_diameter_m', 0.043)       # confirmed (manually measured)
        self.declare_parameter('ticks_per_rev', 360)             # confirmed (LargeMotor)
        self.declare_parameter('wheelbase_m', 0.15)              # confirmed (manually measured)
        self.declare_parameter('steering_center_tick', 90)        # NOTE: assumed from homing, not separately re-measured
        self.declare_parameter('steering_max_tick_dev', 89)       # NOTE: assumed from tick range, not separately re-measured
        self.declare_parameter('steering_max_angle_deg', 47.0)   # confirmed (manually measured)
        self.declare_parameter('encoder_sign', -1)               # Motor/encoder counts opposite to vehicle fw

        self.wheel_circumference = math.pi * self.get_parameter('wheel_diameter_m').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.wheelbase = self.get_parameter('wheelbase_m').value
        self.steer_center = self.get_parameter('steering_center_tick').value
        self.steer_max_dev = self.get_parameter('steering_max_tick_dev').value
        self.steer_max_angle = math.radians(
            self.get_parameter('steering_max_angle_deg').value
        )
        self.encoder_sign = self.get_parameter('encoder_sign').value

        # --- State [x, y, psi] ---
        self.x = 0.0
        self.y = 0.0
        self.psi = 0.0
        self.last_enc_ticks = None
        self.last_time = None
        self.steering_ticks = self.steer_center  # default: straight ahead

        # --- ROS I/O ---
        self.create_subscription(Int32, '/ev3/encoder_r', self.encoder_callback, 10)
        self.create_subscription(Int32, '/ev3/steering', self.steering_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'ev3_odometry_node started. wheelbase={self.wheelbase:.3f} m (measured), '
            f'steering_max_angle={math.degrees(self.steer_max_angle):.1f} deg (measured)'
        )

    def steering_callback(self, msg: Int32):
        self.steering_ticks = msg.data

    def ticks_to_delta(self, ticks: int) -> float:
        """Linear tick-to-angle mapping using the measured max steering angle.
        NOTE: assumes symmetric, linear steering response between center and
        max deflection. Only verified at the endpoints -- if the report needs
        proof of linearity, measure a few intermediate tick values too.
        """
        dev = ticks - self.steer_center
        dev = max(-self.steer_max_dev, min(self.steer_max_dev, dev))
        return (dev / self.steer_max_dev) * self.steer_max_angle

    def encoder_callback(self, msg: Int32):
        now = self.get_clock().now()
        ticks = msg.data

        if self.last_enc_ticks is None:
            self.last_enc_ticks = ticks
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        dticks = ticks - self.last_enc_ticks
        distance = self.encoder_sign * (dticks / self.ticks_per_rev) * self.wheel_circumference
        v = distance / dt
        delta = self.ticks_to_delta(self.steering_ticks)

        # Kinematic bicycle model, reference point = rear axle
        self.x += v * math.cos(self.psi) * dt
        self.y += v * math.sin(self.psi) * dt
        self.psi += (v / self.wheelbase) * math.tan(delta) * dt
        self.psi = math.atan2(math.sin(self.psi), math.cos(self.psi))  # normalize to [-pi, pi]

        self.publish_odometry(now, v, delta)

        self.last_enc_ticks = ticks
        self.last_time = now

    def publish_odometry(self, stamp, v, delta):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.psi)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = (v / self.wheelbase) * math.tan(delta)
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = yaw_to_quaternion(self.psi)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Ev3OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
