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
            executable="trj_planner",
            name="kr210_trj_planner",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {"trajectory_execution.execution_duration_monitoring": False},
                {"trajectory_execution.allowed_start_tolerance": 0.01},
                {"trajectory_execution.trajectory_smoother": "ruckig"},
                {"trajectory_execution.trajectory_smoother_parameters": {
                    "max_velocity_scaling_factor": 0.7,
                    "max_acceleration_scaling_factor": 0.7
                }},
            ],
            respawn=False
        )
    ])

