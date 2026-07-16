# ev3-odometry

ROS2-based odometry architecture for a CLEV3R Car EV3 platform with front-axle steering and external IMU node.

## System Overview

- **EV3 Brick** (ev3dev) — motor control, encoder readout, TCP server
- **STM32F401RE + X-Nucleo-IDW04A1** — IMU data acquisition (LSM6DS33), TCP stream over WiFi
- **Host PC** (Ubuntu 22.04, ROS2 Humble) — ROS2 nodes, odometry computation
- **Sony DualSense** — manual control via USB

## ROS2 Packages

| Package | Description | Status |
|---------|-------------|--------|
| `ev3_bridge` | EV3 TCP socket → `/ev3/encoder_r`, `/ev3/steering`, `/ev3/battery` + `controller_node` | ✅ working |
| `imu_bridge` | STM32 TCP stream → `/imu/raw` (`sensor_msgs/Imu`) at 50 Hz | ✅ working |
| `ev3_odometry` | Bicycle model → `/odom` (`nav_msgs/Odometry`) | 🔄 in progress |
| `ev3_bringup` | Launch files for the full system | 🔄 in progress |

## ROS2 Topics

| Topic | Type | Publisher |
|-------|------|-----------|
| `/ev3/encoder_r` | `std_msgs/Int32` | `ev3_bridge_node` |
| `/ev3/steering` | `std_msgs/Int32` | `ev3_bridge_node` |
| `/ev3/battery` | `sensor_msgs/BatteryState` | `ev3_bridge_node` |
| `/ev3/cmd` | `std_msgs/String` | `controller_node` |
| `/imu/raw` | `sensor_msgs/Imu` | `imu_bridge_node` |
| `/odom` | `nav_msgs/Odometry` | `ev3_odometry_node` |

## Hardware

| Component | Function |
|-----------|----------|
| EV3 MediumMotor (OUTPUT_A) | Steering — 0–179 ticks, 90 = straight |
| EV3 LargeMotor (OUTPUT_B/C) | Rear-wheel drive — 360 ticks/rev, ø 43 mm |
| LSM6DS33 IMU | Accel ±4g, Gyro 500 dps, 50 Hz via I²C |
| STM32F401RE | IMU readout + TCP/WiFi stream |
| Sony DualSense | R2 = forward, L2 = backward, left stick = steer |

## Quick Start

### EV3 Server
```bash
# On EV3 (ev3dev):
python3 ev3_server.py
```

### Host PC
```bash
# Terminal 1 — IMU bridge
source ~/ev3-odometry/ros2_ws/install/setup.bash
ros2 run imu_bridge imu_bridge_node

# Terminal 2 — EV3 bridge
ros2 run ev3_bridge ev3_bridge_node

# Terminal 3 — Controller
ros2 run ev3_bridge controller_node
```

### Verify topics
```bash
ros2 topic echo /imu/raw
ros2 topic echo /ev3/encoder_r
ros2 topic echo /ev3/steering
```

## Repository Structure

```
ev3-odometry/
├── ros2_ws/src/
│   ├── ev3_bridge/       # EV3 bridge + controller node
│   ├── imu_bridge/       # IMU bridge node
│   ├── ev3_odometry/     # Odometry node (in progress)
│   └── ev3_bringup/      # Launch files (in progress)
├── ev3/
│   └── ev3_server.py     # Runs on EV3 Brick
└── README.md
```

## Version

Current: "v0.2.0"
