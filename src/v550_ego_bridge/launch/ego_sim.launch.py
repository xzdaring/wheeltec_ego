"""V550 EGO 仿真入口：统一启动 Gazebo、SLAM、RViz、三个适配器和规划器。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bridge = get_package_share_directory("v550_ego_bridge")
    navigation = get_package_share_directory("v550_navigation")
    # 两层安全半径不同：0.20m包围实体，0.25m额外留安全余量；各节点共用配置。
    safety = {"collision_radius": 0.20, "inflation_radius": 0.25}
    return LaunchDescription([
        Node(package="v550_ego_bridge", executable="global_route.py", output="screen",
             parameters=[safety, {"use_sim_time": True}]),
        # 无界面回归测试与桌面运行共用启动入口，避免验证另一套配置。
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # 本入口只连接仿真 /cmd_vel；禁用 Nav2 保证不与其他控制器抢发速度。
        Node(package="v550_ego_bridge", executable="trajectory_follower.py", output="screen",
             parameters=[os.path.join(bridge, "config", "mecanum.yaml"), {"use_sim_time": True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation, "launch", "simulation_nav.launch.py")),
            # 禁用 Nav2，避免与 EGO 的 NavigateToPose 服务争用同一个 action。
            launch_arguments={"start_nav2": "false", "start_slam": "true",
                              "use_rviz": LaunchConfiguration("use_rviz"), "use_gui": LaunchConfiguration("use_gui"), "use_sim_time": "true"}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bridge, "launch", "bridge.launch.py"))),
        Node(package="ego_planner", executable="motion_plan", output="screen",
             parameters=[{"use_sim_time": True, "inflation_radius": 0.25}]),
    ])
