import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('alicia_duo_descriptions')
    
    # Define paths
    # Change the path to point to your .urdf.xacro file
    xacro_file = os.path.join(pkg_dir, 'urdf', 'alicia_duo_with_gripper.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'config', 'alicia.rviz')
    
    # Declare arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value='',
        description='Model parameter'
    )
    
    # Configure nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # Use the 'Command' substitution to process the xacro file
        parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])