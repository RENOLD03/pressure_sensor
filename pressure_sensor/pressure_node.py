#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import ms5837

G = 9.80665  # m/s^2

class PressureNode(Node):
    def __init__(self):
        super().__init__('pressure_node')

        # ---- Parameters ----
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('fluid_density', 997.0)       # kg/m^3 (freshwater)
        self.declare_parameter('zero_on_startup', True)
        self.declare_parameter('baseline_samples', 60)
        self.declare_parameter('surface_pressure_mbar', 1013.25)

        bus = int(self.get_parameter('i2c_bus').value)
        self.fluid_density = float(self.get_parameter('fluid_density').value)
        zero_on_startup = bool(self.get_parameter('zero_on_startup').value)
        self.baseline_samples = int(self.get_parameter('baseline_samples').value)
        cfg_surface_p = float(self.get_parameter('surface_pressure_mbar').value)

        # ---- Sensor ----
        self.sensor = ms5837.MS5837_30BA(bus)
        if not self.sensor.init():
            self.get_logger().error('Could not initialize MS5837 sensor.')
            raise RuntimeError('MS5837 init failed')

        # Optional (the library uses this for its own depth(), we compute ours anyway)
        try:
            self.sensor.setFluidDensity(self.fluid_density)
        except Exception:
            pass

        # ---- Baseline (P0) ----
        if zero_on_startup:
            self.p0_mbar = self._measure_baseline(self.baseline_samples)
            self.get_logger().info(f'Zeroed at startup: P0 = {self.p0_mbar:.2f} mbar')
        else:
            self.p0_mbar = cfg_surface_p
            self.get_logger().info(f'Using configured surface pressure: P0 = {self.p0_mbar:.2f} mbar')

        # ---- Publishers ----
        self.pressure_pub = self.create_publisher(FluidPressure, 'pressure', 10)
        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)
        self.depth_pub = self.create_publisher(Float32, 'depth', 10)
        self.altitude_pub = self.create_publisher(Float32, 'altitude', 10)

        # ---- Services ----
        self.zero_srv = self.create_service(Trigger, 'zero_depth', self.handle_zero_depth)

        # ---- Timer ----
        self.timer = self.create_timer(0.5, self.read_sensor)
        self.get_logger().info('Pressure sensor node started')

    def _measure_baseline(self, samples: int) -> float:
        vals = []
        for _ in range(max(5, samples)):
            if self.sensor.read():
                vals.append(self.sensor.pressure())  # mbar (absolute)
            time.sleep(0.02)
        return sum(vals) / len(vals)

    def handle_zero_depth(self, request, response):
        self.p0_mbar = self._measure_baseline(self.baseline_samples)
        response.success = True
        response.message = f'Baseline reset: P0 = {self.p0_mbar:.2f} mbar'
        self.get_logger().info(response.message)
        return response

    def read_sensor(self):
        if not self.sensor.read():
            self.get_logger().warn('Sensor read failed')
            return

        pressure_mbar = self.sensor.pressure()
        temperature_c = self.sensor.temperature()

        # Calibrated (relative) depth
        depth_m = (pressure_mbar - self.p0_mbar) * 100.0 / (self.fluid_density * G)
        # Clamp tiny negatives to zero
        if abs(depth_m) < 0.02:
            depth_m = 0.0

        # (Optional) library altitude; keep if you find it useful
        try:
            altitude_m = self.sensor.altitude()
        except Exception:
            altitude_m = 0.0

        # Publish pressure (Pa)
        p_msg = FluidPressure()
        p_msg.fluid_pressure = pressure_mbar * 100.0
        self.pressure_pub.publish(p_msg)

        # Publish temperature (°C)
        t_msg = Temperature()
        t_msg.temperature = float(temperature_c)
        self.temp_pub.publish(t_msg)

        # Publish depth (m, positive down)
        d_msg = Float32()
        d_msg.data = float(depth_m)
        self.depth_pub.publish(d_msg)

        # Publish altitude (m, above MSL per library’s model)
        a_msg = Float32()
        a_msg.data = float(altitude_m)
        self.altitude_pub.publish(a_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PressureNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
