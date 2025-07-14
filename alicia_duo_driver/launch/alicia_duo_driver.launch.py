from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch the C++ Alicia Duo driver node."""
    
    alicia_driver_node = Node(
        package='alicia_duo_driver',
        executable='alicia_duo_driver_node',
        name='alicia_duo_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'port': '/dev/ttyUSB0'},
            {'baud_rate': 921600},
            {'debug_mode': True},
            {'servo_count': 9},
            {'rate_limit_sec': 0.01}
        ]
    )

    return LaunchDescription([
        alicia_driver_node
    ])