# ev3-odometry

ROS2-based odometry architecture for a CLEV3R Car EV3 platform with front-axle steering and external IMU node.

## System Overview

- **EV3 Brick** (ev3dev) — motor control, encoder readout, TCP server
- **STM32F401RE + X-Nucleo-IDW04A1** — IMU data acquisition (LSM6DS33), TCP stream over WiFi
- **Host PC** (Ubuntu 22.04, ROS2 Humble) — ROS2 nodes, odometry computation, Kalman yaw fusion
- **Sony DualSense** — manual control via USB

## ROS2 Packages

| Package | Description | Status |
|---------|-------------|--------|
| `ev3_bridge` | EV3 TCP socket → `/ev3/encoder_r`, `/ev3/steering`, `/ev3/battery` + `controller_node` | ✅ working |
| `imu_bridge` | STM32 TCP stream → `/imu/raw` (`sensor_msgs/Imu`) | ✅ working |
| `ev3_odometry` | Bicycle-model odometry (`ev3_odometry_node`) + encoder/gyro yaw fusion (`yaw_kalman_node`) | ✅ working |
| `ev3_bringup` | Launch file for the full system | ✅ working |

## ROS2 Topics

| Topic | Type | Publisher |
|-------|------|-----------|
| `/ev3/encoder_r` | `std_msgs/Int32` | `ev3_bridge_node` |
| `/ev3/steering` | `std_msgs/Int32` | `ev3_bridge_node` |
| `/ev3/battery` | `sensor_msgs/BatteryState` | `ev3_bridge_node` |
| `/ev3/cmd` | `std_msgs/String` | `controller_node` |
| `/imu/raw` | `sensor_msgs/Imu` | `imu_bridge_node` |
| `/odom` | `nav_msgs/Odometry` | `ev3_odometry_node` |
| `/yaw/encoder` | `std_msgs/Float32` | `yaw_kalman_node` — raw encoder-derived yaw, comparison only |
| `/yaw/imu` | `std_msgs/Float32` | `yaw_kalman_node` — pure gyro integration, drifts, comparison only |
| `/yaw/kalman` | `std_msgs/Float32` | `yaw_kalman_node` — fused yaw estimate |
| `/odom_filtered` | `nav_msgs/Odometry` | `yaw_kalman_node` — fused yaw + dead-reckoned (x, y); position itself is not part of the Kalman state |

`ev3_odometry_node` also broadcasts the `odom → base_link` TF transform. `yaw_kalman_node` does **not** broadcast a TF, to avoid two nodes fighting over the same transform — `/odom_filtered` is data-only.

## Hardware

| Component | Function |
|-----------|----------|
| EV3 MediumMotor (OUTPUT_A) | Steering — 0–179 ticks, 90 = straight |
| EV3 LargeMotor (OUTPUT_B/C) | Rear-wheel drive — 360 ticks/rev, ø 43 mm |
| LSM6DS33 IMU | Accel ±4g, Gyro ±500 dps, 104 Hz via I²C |
| STM32F401RE | IMU readout + TCP/WiFi stream |
| Sony DualSense | R2 = forward, L2 = backward, left stick = steer |

## Quick Start

### EV3 Server
```bash
# On EV3 (ev3dev):
python3 ev3_server.py
```

### Host PC — full system via launch
```bash
source ~/ev3-odometry/ros2_ws/install/setup.bash
ros2 launch ev3_bringup ev3_bringup_launch.py
```
Starts `ev3_bridge_node`, `controller_node`, `imu_bridge_node`, `ev3_odometry_node`, and `yaw_kalman_node` with the final tuned Kalman parameters (`Q = 2.4e-7`, `R = 5.0`, see semester report for derivation).

### Host PC — manual (for debugging individual nodes)
```bash
# Terminal 1 — IMU bridge
ros2 run imu_bridge imu_bridge_node
# Terminal 2 — EV3 bridge
ros2 run ev3_bridge ev3_bridge_node
# Terminal 3 — Controller
ros2 run ev3_bridge controller_node
# Terminal 4 — Odometry
ros2 run ev3_odometry ev3_odometry_node
# Terminal 5 — Yaw fusion
ros2 run ev3_odometry yaw_kalman_node
```
Note: when run manually (not via launch), `yaw_kalman_node` uses its declared parameter defaults, which are also set to the final tuned values.

### Verify topics
```bash
ros2 topic echo /imu/raw
ros2 topic echo /ev3/encoder_r
ros2 topic echo /ev3/steering
ros2 topic echo /yaw/kalman
ros2 topic echo /odom_filtered
```

## Repository Structure

```
ev3-odometry/
├── ros2_ws/src/
│   ├── ev3_bridge/       # EV3 bridge + controller node
│   ├── imu_bridge/       # IMU bridge node
│   ├── ev3_odometry/     # Odometry node + yaw Kalman fusion
│   └── ev3_bringup/      # Launch file
├── ev3/
│   └── ev3_server.py     # Runs on EV3 Brick
└── README.md
```

## Version

Current: `v1.0.0`