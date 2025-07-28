import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Float32
import ms5837

class PressureNode(Node):
    def __init__(self):
        super().__init__('pressure_node')

        self.sensor = ms5837.MS5837_30BA()
        if not self.sensor.init():
            self.get_logger().error("Failed to initialize Bar30 sensor.")
            raise RuntimeError("Sensor init failed")

        self.sensor.setFluidDensity(997)  # freshwater

        self.pressure_pub = self.create_publisher(FluidPressure, '/bar30/pressure', 10)
        self.temp_pub = self.create_publisher(Temperature, '/bar30/temperature', 10)
        self.depth_pub = self.create_publisher(Float32, '/bar30/depth', 10)
        self.alt_pub = self.create_publisher(Float32, '/bar30/altitude', 10)

        self.timer = self.create_timer(1.0, self.read_sensor)  # 1 Hz

    def read_sensor(self):
        if self.sensor.read():
            # Pressure
            pressure_msg = FluidPressure()
            pressure_msg.fluid_pressure = self.sensor.pressure() * 100.0  # mbar → Pa
            pressure_msg.variance = 0.0
            self.pressure_pub.publish(pressure_msg)

            # Temperature
            temp_msg = Temperature()
            temp_msg.temperature = self.sensor.temperature()
            temp_msg.variance = 0.0
            self.temp_pub.publish(temp_msg)

            # Depth
            depth_msg = Float32()
            depth_msg.data = self.sensor.depth()
            self.depth_pub.publish(depth_msg)

            # Altitude
            alt_msg = Float32()
            alt_msg.data = self.sensor.altitude()
            self.alt_pub.publish(alt_msg)

            self.get_logger().info(
                f"Pressure: {pressure_msg.fluid_pressure:.2f} Pa, Temp: {temp_msg.temperature:.2f} °C, "
                f"Depth: {depth_msg.data:.3f} m, Alt: {alt_msg.data:.2f} m"
            )
        else:
            self.get_logger().warn("Failed to read from sensor.")

def main(args=None):
    rclpy.init(args=args)
    node = PressureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
