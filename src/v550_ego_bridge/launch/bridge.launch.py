import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """一次启动状态和感知两个适配器，组成 V550 -> EGO 的完整输入桥。"""
    package_share = get_package_share_directory("v550_ego_bridge")
    state_parameters = os.path.join(package_share, "config", "state_adapter.yaml")
    scan_parameters = os.path.join(package_share, "config", "scan_to_obstacles.yaml")
    goal_parameters = os.path.join(package_share, "config", "goal_to_path.yaml")

    return LaunchDescription([
        # 把里程计位姿转换成 EGO Planner 使用的 /current_pose。
        Node(
            package="v550_ego_bridge",
            executable="state_adapter",
            name="v550_ego_state_adapter",
            output="screen",
            parameters=[state_parameters, {"use_sim_time": True}],
        ),
        # 把激光极坐标数据转换成 map 坐标系下的障碍物点云。
        Node(
            package="v550_ego_bridge",
            executable="scan_to_obstacles",
            name="scan_to_obstacles",
            output="screen",
            parameters=[scan_parameters, {"use_sim_time": True}],
        ),
        # 将 RViz 目标位姿和当前位置连接成 EGO 所需的全局参考路径。
        Node(
            package="v550_ego_bridge",
            executable="goal_to_path",
            name="goal_to_path",
            output="screen",
            parameters=[goal_parameters, {"use_sim_time": True}],
        ),
    ])
