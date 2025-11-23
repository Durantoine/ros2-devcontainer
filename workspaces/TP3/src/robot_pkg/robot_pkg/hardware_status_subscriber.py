import rclpy
from rclpy.node import Node
from interface_pkg.msg import HardwareStatus


class HardwareStatusSubscriber(Node):
    def __init__(self):
        super().__init__("hardware_status_subscriber")
        self.subscription = self.create_subscription(
            HardwareStatus, "hardware_status", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.get_logger().info(
            f"I heard: temp={msg.temperature_celsius}, motors_ready={msg.are_motors_ready}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
