# ev3-odometry
ROS2-based odometry architecture for a CLEV3R Car EV3 platform.

## Packages
- `ev3_bridge` — EV3 TCP socket → ROS2 topics
- `imu_bridge` — STM32 UDP → sensor_msgs/Imu
- `ev3_odometry` — Bicycle model → nav_msgs/Odometry
- `ev3_bringup` — Launch files
