import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Arguments for C++ Driver ---
    declare_port_arg = DeclareLaunchArgument(
        'port', default_value='ttyUSB0', description='Serial port device name')
    declare_baudrate_arg = DeclareLaunchArgument(
        'baudrate', default_value='921600', description='Serial port baud rate')
    declare_timeout_arg = DeclareLaunchArgument(
        'timeout_ms', default_value='1000', description='Serial read timeout ms')

    # --- Arguments for Python Nodes ---
    declare_py_debug_arg = DeclareLaunchArgument(
        'py_debug', default_value='false', description='Enable debug for Python nodes')
    declare_servo_count_arg = DeclareLaunchArgument(
        'servo_count', default_value='9', description='Number of servos expected')
    declare_rate_limit_arg = DeclareLaunchArgument(
        'rate_limit_sec', default_value='0.01', description='Rate limit for joint state publisher')

    # --- Node Definitions ---

    # C++ Serial Driver Node
    serial_driver_node = Node(
        package='alicia_duo_driver', executable='alicia_duo_driver_node', name='serial_driver', output='screen',
        parameters=[{'port': LaunchConfiguration('port'),
                     'baudrate': LaunchConfiguration('baudrate'),
                     'debug_mode': LaunchConfiguration('py_debug'), # Link to common debug flag
                     'timeout_ms': LaunchConfiguration('timeout_ms')}]
    )

    # Python Serial Dispatcher Node
    serial_dispatcher_node = Node(
        package='alicia_duo_driver', executable='serial_dispatcher_node.py', name='serial_dispatcher', output='screen',
        parameters=[{'debug_mode': LaunchConfiguration('py_debug')}]
    )

    # Python Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='alicia_duo_driver', executable='joint_state_publisher_node.py', name='joint_state_publisher', output='screen',
        parameters=[{'debug_mode': LaunchConfiguration('py_debug'),
                     'servo_count': LaunchConfiguration('servo_count'),
                     'rate_limit_sec': LaunchConfiguration('rate_limit_sec')}]
    )

    # Python Arm Control Node <--- 新增节点定义
    arm_control_node = Node(
        package='alicia_duo_driver',
        executable='arm_control_node.py', # 脚本文件名
        name='arm_control', # 节点名
        output='screen',
        parameters=[{
            'debug_mode': LaunchConfiguration('py_debug'),
            'servo_count': LaunchConfiguration('servo_count'),
            # Add other params for arm_control_node if needed
        }]
    )

    # --- Launch Description Assembly ---
    return LaunchDescription([
        # Declare Arguments
        declare_port_arg,
        declare_baudrate_arg,
        declare_timeout_arg,
        declare_py_debug_arg,
        declare_servo_count_arg,
        declare_rate_limit_arg,

        LogInfo(msg=['Launching nodes... Port: ', LaunchConfiguration('port'),
                     ', Baudrate: ', LaunchConfiguration('baudrate'),
                     ', PyDebug: ', LaunchConfiguration('py_debug')]),

        # Launch Nodes
        serial_driver_node,
        serial_dispatcher_node,
        joint_state_publisher_node,
        arm_control_node, # <--- 将新节点添加到启动列表
    ])