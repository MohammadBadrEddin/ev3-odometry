from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ev3_bridge',
            executable='ev3_bridge_node',
            name='ev3_bridge_node',
            output='screen',
        ),
        Node(
            package='ev3_bridge',
            executable='controller_node',
            name='controller_node',
            output='screen',
        ),
        Node(
            package='ev3_odometry',
            executable='ev3_odometry_node',
            name='ev3_odometry_node',
            output='screen',
            parameters=[{
                'wheelbase_m': 0.16,
                'steering_max_angle_deg': 47.0,
            }],
        ),

        # imu_bridge_node's TCP accept() call runs on a background daemon
        # thread (see receive_loop), so it does NOT block rclpy.spin() or
        # the rest of the launch if the STM32 is not yet connected -- it
        # just logs "waiting for STM32" and idles. Safe to always launch.
        Node(
            package='imu_bridge',
            executable='imu_bridge_node',
            name='imu_bridge_node',
            output='screen',
        ),
        Node(
            package='ev3_odometry',
            executable='yaw_kalman_node',
            name='yaw_kalman_node',
            output='screen',
            parameters=[{
                'Q'2.4e-7: ,
                'R': 5.0,
                'initial_P': 1.0,
            }],
        ),
    ])
