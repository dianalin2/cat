import rclpy
from teleop_node.node import TeleopNode

def main(args=None):
    rclpy.init(args=args)

    node = TeleopNode()

    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()
