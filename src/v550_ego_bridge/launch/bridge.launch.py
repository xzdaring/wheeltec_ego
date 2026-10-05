import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument  # 允许实车和仿真入口选择时间源。
from launch.substitutions import LaunchConfiguration  # 将父launch的参数传递给节点。
from launch_ros.parameter_descriptions import ParameterValue  # 明确布尔类型，避免字符串被当作时间开关。


def generate_launch_description():
    """一次启动状态和感知两个适配器，组成 V550 -> EGO 的完整输入桥。"""
    package_share = get_package_share_directory("v550_ego_bridge")
    state_parameters = os.path.join(package_share, "config", "state_adapter.yaml")
    scan_parameters = os.path.join(package_share, "config", "scan_to_obstacles.yaml")
    goal_parameters = os.path.join(package_share, "config", "goal_to_path.yaml")

    clock = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)  # 实车不等待/clock。
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),  # 保持原仿真入口的默认行为。
        # 把里程计位姿转换成 EGO Planner 使用的 /current_pose。
        Node(
            package="v550_ego_bridge",
            executable="state_adapter",
            name="v550_ego_state_adapter",
            output="screen",
            parameters=[state_parameters, {"use_sim_time": clock}],  # 覆盖yaml时间源，实车传false，仿真传true。
        ),
        # 把激光极坐标数据转换成 map 坐标系下的障碍物点云。
        Node(
            package="v550_ego_bridge",
            executable="scan_to_obstacles",
            name="scan_to_obstacles",
            output="screen",
            parameters=[scan_parameters, {"use_sim_time": clock}],  # 覆盖yaml时间源，实车传false，仿真传true。
        ),
        # 将 RViz 目标位姿和当前位置连接成 EGO 所需的全局参考路径。
        Node(
            package="v550_ego_bridge",
            executable="goal_to_path",
            name="goal_to_path",
            output="screen",
            parameters=[goal_parameters, {"use_sim_time": clock}],  # 覆盖yaml时间源，实车传false，仿真传true。
        ),
    ])
