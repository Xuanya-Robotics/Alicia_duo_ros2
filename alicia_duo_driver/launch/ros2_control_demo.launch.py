import os
import yaml
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

    # Get robot description (URDF)
    robot_description_content = Command(
        ['xacro ', os.path.join(pkg_dir, 'config', 'alicia_duo_descriptions_real.urdf.xacro')]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Get semantic robot description (SRDF)
    srdf_path = os.path.join(pkg_dir, 'config', 'alicia_duo_descriptions.srdf')
    with open(srdf_path, 'r') as f:
        srdf_content = f.read()
    robot_description_semantic = {'robot_description_semantic': srdf_content}

    # Get Kinematics parameters
    kinematics_yaml_path = os.path.join(pkg_dir, 'config', 'kinematics.yaml')
    with open(kinematics_yaml_path, 'r') as f:
        kinematics_dict = yaml.safe_load(f)

    # Get MoveIt controller parameters
    moveit_controllers_path = os.path.join(pkg_dir, 'config', 'moveit_controllers.yaml')
    with open(moveit_controllers_path, 'r') as f:
        moveit_controllers_dict = yaml.safe_load(f)

    # ros2_control parameters
    robot_controllers = os.path.join(pkg_dir, 'config', 'ros2_controllers.yaml')

    ompl_planning_pipeline_config_path = os.path.join(pkg_dir, 'config', 'ompl_planning.yaml') # <-- ADD THIS
    with open(ompl_planning_pipeline_config_path, 'r') as f:
        ompl_planning_pipeline_config = yaml.safe_load(f) 


    # ===================================================================================
    # |                                 Nodes                                           |
    # ===================================================================================

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen',
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
        name="move_group",  # <-- EXPLICITLY NAME THE NODE
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_dict,
            moveit_controllers_dict,
            ompl_planning_pipeline_config,
        ],
    )

    # RViz node
    rviz_config = os.path.join(pkg_dir, 'config', 'moveit.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_dict,
        ],
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        move_group_node,
        rviz_node,
    ])