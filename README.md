# Pressure Sensor ROS 2 Node

This ROS 2 package reads data from the Blue Robotics Bar30 (MS5837-30BA) pressure sensor over I²C and publishes:

- Pressure (`/bar30/pressure`) — `sensor_msgs/FluidPressure`
- Temperature (`/bar30/temperature`) — `sensor_msgs/Temperature`
- Depth (`/bar30/depth`) — `std_msgs/Float32`
- Altitude (`/bar30/altitude`) — `std_msgs/Float32`


---

## Detecting the Sensor (I²C Bus Check)

To verify the sensor is connected correctly via I²C:

```bash
sudo apt install i2c-tools
i2cdetect -l               # Lists available I²C buses
sudo i2cdetect -y 1        # Scan bus 1 (most common on Jetson)
---

## Wiring

| Sensor Wire | GPS_JET Pin | Signal |
|-------------|-------------|--------|
| Red         | 1           | 5V     |
| Green       | 4           | SCL    |
| White       | 5           | SDA    |
| Black       | 6           | GND    |

Make sure your device uses 3.3V I²C logic (Bar30 does).

## Dependencies

Install dependencies:

```bash
sudo apt install python3-smbus i2c-tools
pip3 install ms5837
