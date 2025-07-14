
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    parameters=[{"publish_rate": 500}]  # 修改频率
)

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("alicia_duo_descriptions", package_name="alicia_duo_moveit").to_moveit_configs()
    return generate_demo_launch(moveit_config)

