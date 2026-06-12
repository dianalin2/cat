import rclpy
from rclpy.node import Node
from feedback_node.node import FeedbackNode

def main(args=None):
    rclpy.init(args=args)

    node = FeedbackNode()

    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()
