from rclpy.node import Node
from msgs.msg import EncoderFeedback
from rclpy.qos import QoSProfile
from serial_node.serial import SerialHandler
from serial_node.constants import FEEDBACK_PACKETS

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

        self.encoder_publisher = self.create_publisher(EncoderFeedback, 'encoder_feedback', QoSProfile(depth=10))
        self.timer = self.create_timer(1.0 / RECV_HZ, self.read_serial_data)

        self.feedback_packet_map = {(packet['header'], packet['length']): packet for packet in FEEDBACK_PACKETS}
        self.get_logger().info(f"Feedback Packet Map: {self.feedback_packet_map}")
        
        self.get_logger().info("SerialNode has been initialized.")
        
    def read_serial_data(self):
        packet = self.serial_handler.read(logger=self.get_logger())

        if not packet:
            return
        
        self.parse_publish_serial_data(*packet)
    
    def parse_publish_serial_data(self, header, data):
        key = (header, len(data))
        if key not in self.feedback_packet_map:
            self.get_logger().warning(f"Received unknown packet header: {hex(header)}")
            return None

        packet_info = self.feedback_packet_map.get(key, None)
        if not packet_info:
            self.get_logger().warning(f"Received packet with header {hex(header)} but unexpected length: {len(data)} bytes.")
            return None

        if packet_info['msg_type'] == 'EncoderFeedback':
            parsed_data = packet_info['parser'](data)
            msg = EncoderFeedback()
            msg.name = parsed_data['name']
            msg.position = parsed_data['position']

            self.encoder_publisher.publish(msg)
            self.get_logger().debug(f"Published EncoderFeedback: {msg.name} - {msg.position}")
            return msg
        else:
            self.get_logger().warning(f"No parser/publisher implemented for message type: {packet_info['msg_type']}")
            return None
