import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # ===================================================================================
    # |                           Path and File Definitions                             |
    # ===================================================================================

    pkg_dir = get_package_share_directory('alicia_duo_moveit')

    # Get robot description
    robot_description_content = Command(
        ['xacro ', os.path.join(pkg_dir, 'config', 'alicia_duo_descriptions_real.urdf.xacro')]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Get controller config
    robot_controllers = os.path.join(pkg_dir, 'config', 'ros2_controllers.yaml')

    # MoveIt config
    moveit_controllers = os.path.join(pkg_dir, 'config', 'moveit_controllers.yaml')
    kinematics_yaml = os.path.join(pkg_dir, 'config', 'kinematics.yaml')
    srdf = os.path.join(pkg_dir, 'config', 'alicia_duo_descriptions.srdf')

    # MoveIt parameters (using MoveItConfigsBuilder is preferred, but here's a manual way)
    moveit_params = [
        robot_description,
        {'robot_description_semantic': open(srdf).read()},
        {'robot_description_kinematics': kinematics_yaml},
    ]


    # ===================================================================================
    # |                                 Actions and Nodes                               |
    # ===================================================================================
    # Nodes
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen',
        # arguments=['--ros-args', '--log-level', 'ros2_control_node.AliciaHardwareInterface:=debug'],
        # prefix=['xterm -e gdb -ex run --args']
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['alicia_controller', '--controller-manager', '/controller_manager'],
    )

    # ===================================================================================
    # |                            Lifecycle Management                                 |
    # ===================================================================================

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )



    # MoveIt move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=moveit_params,
    )

    # RViz node (optional)
    rviz_config = os.path.join(pkg_dir, 'config', 'moveit.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=moveit_params,
    )


    return LaunchDescription([
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        move_group_node,
        rviz_node,
    ])