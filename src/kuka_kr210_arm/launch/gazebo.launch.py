import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory("kuka_kr210_arm")
    urdf_file = os.path.join(package_share_dir, "urdf", "kr210.urdf")

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gz", "sim", "-r"],
                output="screen",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", "kuka_kr210_arm", "-file", urdf_file],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ),
        ]
    )

