from rclpy.node import Node
from msgs.msg import SerialMessage
from rclpy.qos import QoSProfile
from serial_node.serial import SerialHandler

RECV_HZ = 20

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('mock_serial', 0)
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        mock_serial = self.get_parameter('mock_serial').get_parameter_value().integer_value != 0
        self.get_logger().info(f"SerialNode parameters: port={port}, baudrate={baudrate}, mock_serial={mock_serial}")

        self.serial_handler = SerialHandler(port=port, baudrate=baudrate, mock_serial=mock_serial)

        self.serial_publisher = self.create_publisher(SerialMessage, 'serial_data', QoSProfile(depth=10))
        self.timer = self.create_timer(1.0 / RECV_HZ, self.read_serial_data)
        
        self.get_logger().info("SerialNode has been initialized.")
        
    def read_serial_data(self):
        data = self.serial_handler.read(logger=self.get_logger())

        if not data:
            return
        
        header, data = data

        msg = SerialMessage()
        msg.length = len(data)
        msg.header = header
        msg.payload = data
        self.serial_publisher.publish(msg)
        self.get_logger().debug(f"Published serial data: {msg.payload}")
