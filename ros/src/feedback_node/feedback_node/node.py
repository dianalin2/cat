from rclpy.node import Node
from msgs.msg import EncoderFeedback
from rclpy.qos import QoSProfile

RECV_HZ = 20

class FeedbackNode(Node):
    def __init__(self):
        super().__init__('feedback_node')

        self.encoder_subscriber = self.create_subscription(
            EncoderFeedback,
            'encoder_feedback',
            self.encoder_callback,
            QoSProfile(depth=10)
        )

        self.get_logger().info("FeedbackNode has been initialized.")

    def encoder_callback(self, msg):
        self.get_logger().info(f"Received encoder feedback data: {[f'{name}: {position:.2f}' for name, position in zip(msg.name, msg.position)]}")
        
    