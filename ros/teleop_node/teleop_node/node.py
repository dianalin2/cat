from rclpy.node import Node
from msgs.msg import Teleop, MotorCommands

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.teleop_publisher = self.create_subscription(Teleop, 'teleop', self.teleop_callback, 10)
        self.motor_command_publisher = self.create_publisher(MotorCommands, 'motor_commands', 10)
        self.get_logger().info("TeleopNode has been initialized.")

    def teleop_callback(self, msg):
        self.get_logger().info(f"Received teleop data: base={msg.base:.2f}, elbow={msg.elbow:.2f}")
        motor_commands = MotorCommands()
        motor_commands.motor_commands = self.calculate_motor_commands(msg.base, msg.elbow)
        self.motor_command_publisher.publish(motor_commands)
        self.get_logger().debug(f"Published motor commands: {motor_commands.motor_commands}")
    
    def map_value(self, value, in_min, in_max, out_min, out_max):
        # Map a value from one range to another
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        
    def calculate_motor_commands(self, base, elbow):
        # Placeholder for actual motor command calculation logic
        # This should be replaced with the real kinematics or control algorithm
        base_value = self.map_value(base, -180, 180, 0, 255)  # Map base angle to motor command range
        elbow_value = self.map_value(elbow, -180, 180, 0, 255)  # Map elbow angle to motor command range
        return [int(base_value), int(elbow_value)] * 4
