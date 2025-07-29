# Pressure Sensor (MS5837) ROS 2 Package

This ROS 2 package interfaces with the **MS5837 pressure sensor** to provide:

- Fluid pressure (`sensor_msgs/FluidPressure`)
- Temperature (`sensor_msgs/Temperature`)
- Depth and altitude (`std_msgs/Float32`)
- Odometry (`nav_msgs/Odometry`) for EKF use, with **depth as negative z** (ROS convention)

## Features

- Publishes accurate depth and pressure data at 2Hz
- Provides an odometry topic `depth_odom` for use with `robot_localization` EKF
- Built for ROS 2 (tested on Humble and Rolling)

---

## Nodes

### `pressure_node`

Publishes raw sensor readings:

| Topic        | Message Type                 | Description                 |
|--------------|------------------------------|-----------------------------|
| `/pressure`  | `sensor_msgs/FluidPressure`  | Absolute fluid pressure     |
| `/temperature` | `sensor_msgs/Temperature`  | Temperature in Celsius      |
| `/depth`     | `std_msgs/Float32`           | Depth in meters             |
| `/altitude`  | `std_msgs/Float32`           | Altitude above surface      |

---

### `depth_odom_node`

Publishes depth as `nav_msgs/Odometry`, with **negative Z** (required by EKF conventions):

| Topic         | Message Type         | Description                          |
|---------------|----------------------|--------------------------------------|
| `/depth_odom` | `nav_msgs/Odometry`  | Depth in `pose.pose.position.z` (neg)|

---

## Usage

### 1. Clone and Build

```bash
cd ~/sensor/src
git clone https://github.com/RENOLD03/pressure_sensor.git
cd ..
colcon build
source install/setup.bash
