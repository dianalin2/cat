import rclpy
from rclpy.node import Node
from serial_node.node import SerialNode

def main(args=None):
    rclpy.init(args=args)

    node = SerialNode()

    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()
