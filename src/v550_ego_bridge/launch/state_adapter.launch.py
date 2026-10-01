import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("v550_ego_bridge")
    parameters = os.path.join(package_share, "config", "state_adapter.yaml")

    return LaunchDescription([
        Node(
            package="v550_ego_bridge",
            executable="state_adapter",
            name="v550_ego_state_adapter",
            output="screen",
            parameters=[parameters],
        )
    ])
