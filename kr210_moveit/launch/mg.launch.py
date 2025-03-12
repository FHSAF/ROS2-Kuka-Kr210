from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "kuka_kr210_arm", package_name="kr210_moveit"
    ).to_moveit_configs()

    return LaunchDescription([
        generate_move_group_launch(moveit_config),

        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            name="server_endpoint_1",
            output="screen",
            parameters=[{"tcp_ip": "0.0.0.0", "tcp_port": 10000}],
        ),

        Node(
            package="kr210_moveit",
            executable="mover_server",
            name="kr210_mover_server",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                # Include any other relevant parameters
            ],
            respawn=False
        )
    ])

