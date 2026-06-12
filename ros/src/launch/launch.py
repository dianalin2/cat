from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rmw_zenoh_cpp",
            executable="rmw_zenohd",
            name="rmw_zenohd",
            output="screen",
            parameters=[
                {"zenoh_router_port": 7447},
                {"zenoh_router_log_level": "info"}
            ]
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
