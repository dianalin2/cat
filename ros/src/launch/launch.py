from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
        ),
        Node(
            package='serial_node',
            executable='serial_node',
            name='serial_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 115200},
                {'mock_serial': EnvironmentVariable('MOCK_SERIAL', default_value='0')}
            ]
        ),
        Node(
            package='feedback_node',
            executable='feedback_node',
            name='feedback_node',
            output='screen'
        ),
        Node(
            package='teleop_node',
            executable='teleop_node',
            name='teleop_node',
            output='screen'
        )
    ])
