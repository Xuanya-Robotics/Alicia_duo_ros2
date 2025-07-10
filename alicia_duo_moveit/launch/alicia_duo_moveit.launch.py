import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # This is the main launch file for the real robot
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

def launch_setup(context, *args, **kwargs):
    moveit_config = (
        MoveItConfigsBuilder("alicia_duo_descriptions", package_name="alicia_duo_moveit")
        .robot_description(
            file_path="config/alicia_duo_descriptions_real.urdf.xacro",
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )


    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("alicia_duo_moveit"), "config", LaunchConfiguration("rviz_config")]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )


    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("alicia_duo_moveit"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["alicia_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of spawners after ros2_control_node is up
    delay_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ros2_control_node,
            on_exit=[joint_state_broadcaster_spawner, robot_controller_spawner],
        )
    )

    alicia_driver_node = Node(
        package="alicia_duo_driver",
        executable="alicia_duo_driver_node",
        name="alicia_duo_driver_node",
        output="screen",
        parameters=[{"serial_port": "/dev/ttyUSB0"}] # Make sure this port is correct
    )

    nodes_to_start = [
        alicia_driver_node, # Add the driver node here
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        delay_spawners,
    ]

    return nodes_to_start