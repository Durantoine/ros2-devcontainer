import rclpy
from rclpy.node import Node
from interface_pkg.msg import HardwareStatus  # Import custom message


class HardwareStatusPublisher(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")
        self.publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.temp_ = 65

    def timer_callback(self):
        msg = HardwareStatus()
        msg.temperature_celsius = self.temp_
        msg.are_motors_ready = True
        msg.debug_message = f"Temperature is {self.temp_} C"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.debug_message}"')
        self.temp_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
