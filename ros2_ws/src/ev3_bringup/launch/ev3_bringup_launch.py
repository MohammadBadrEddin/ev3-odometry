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
                'wheelbase_m': 0.15,
                'steering_max_angle_deg': 47.0,
            }],
        ),

        # IMU is not physically mounted on the robot yet.
        # Uncomment both nodes below once the STM32 IMU node is installed,
        # powered, and connected over WiFi -- otherwise imu_bridge_node
        # will hang waiting for a TCP connection that never arrives.
        # Node(
        #     package='imu_bridge',
        #     executable='imu_bridge_node',
        #     name='imu_bridge_node',
        #     output='screen',
        # ),
        # Node(
        #     package='ev3_odometry',
        #     executable='yaw_kalman_node',
        #     name='yaw_kalman_node',
        #     output='screen',
        # ),
    ])
