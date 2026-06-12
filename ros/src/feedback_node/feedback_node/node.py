from rclpy.node import Node
from msgs.msg import SerialMessage
from rclpy.qos import QoSProfile

RECV_HZ = 20

class FeedbackNode(Node):
    def __init__(self):
        super().__init__('feedback_node')

        self.serial_subscriber = self.create_subscription(
            SerialMessage,
            'serial_data',
            self.serial_callback,
            QoSProfile(depth=10)
        )

        self.get_logger().info("FeedbackNode has been initialized.")

    def serial_callback(self, msg):
        self.get_logger().info(f"Received serial data: {msg.header} - {bytearray(msg.payload)}")
        
    