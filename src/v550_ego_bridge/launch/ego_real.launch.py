"""V550实车EGO总入口：真实驱动、在线建图、规划与跟踪，使用系统时间。"""
import os  # 根据安装位置拼接路径，避免绑定开发电脑目录。
from ament_index_python.packages import get_package_share_directory  # 从当前overlay查找配置。
from launch import LaunchDescription  # 返回可执行的launch动作列表。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 提供开关并复用实车底层入口。
from launch.substitutions import LaunchConfiguration  # 读取命令行参数。
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 加载Python子launch。
from launch_ros.parameter_descriptions import ParameterValue  # 将命令行规划参数明确转换为浮点数。
from launch_ros.actions import Node  # 启动EGO和控制节点。


def generate_launch_description():  # ros2 launch调用此函数构造启动链。
    bridge = get_package_share_directory("v550_ego_bridge")  # 找到适配器配置及子launch。
    navigation = get_package_share_directory("v550_navigation")  # 使用本工程实车入口，不启动Gazebo。
    safety = {"collision_radius": 0.05, "inflation_radius": 0.10}  # 与当前规划和车体包络保持一致。
    return LaunchDescription([
        DeclareLaunchArgument("planning_horizon", default_value="1.5"),  # 局部规划空间视野，单位米。
        DeclareLaunchArgument("replan_interval", default_value="0.5"),  # 滚动重规划周期，单位秒。  # 全部节点继承终端DDS配置，实车统一使用系统时间。
        DeclareLaunchArgument("bringup_hardware", default_value="true"),  # 已单独启动底盘和雷达时传false避免重复驱动。
        DeclareLaunchArgument("start_slam", default_value="true"),  # 已有在线SLAM时传false，避免重复发布map TF。
        DeclareLaunchArgument("use_rviz", default_value="false"),  # 车机无桌面也能启动；有桌面时可传true。
        IncludeLaunchDescription(  # 复用真实底盘、雷达、扫描归一化及在线SLAM。
            PythonLaunchDescriptionSource(os.path.join(navigation, "launch", "real_navigation.launch.py")),  # 禁止包含simulation_nav。
            launch_arguments={"bringup_hardware": LaunchConfiguration("bringup_hardware"),  # 用户决定是否复用现有硬件。
                              "start_slam": LaunchConfiguration("start_slam"),  # 在线建图无需预先地图。
                              "start_nav2": "false",  # EGO独占导航action和跟踪速度输出。
                              "use_rviz": LaunchConfiguration("use_rviz")}.items()),  # 可选加载现有轨迹显示配置。
        IncludeLaunchDescription(  # 启动状态、激光和导航目标三个适配器。
            PythonLaunchDescriptionSource(os.path.join(bridge, "launch", "bridge.launch.py")),  # 保持两种部署的接口一致。
            launch_arguments={"use_sim_time": "false"}.items()),  # 覆盖适配器yaml中的仿真时间。
        Node(package="v550_ego_bridge", executable="global_route.py", output="screen",  # 根据实时感知生成障碍栅格。
             parameters=[safety, {"use_sim_time": False}]),  # 使用系统时间，不依赖仿真clock。
        Node(package="ego_planner", executable="motion_plan", output="screen",  # EGO负责全局参考和局部优化。
             parameters=[{"use_sim_time": False, "inflation_radius": 0.25, "planning_horizon": ParameterValue(LaunchConfiguration("planning_horizon"), value_type=float), "replan_interval": ParameterValue(LaunchConfiguration("replan_interval"), value_type=float)}]),  # 与代价地图保持相同膨胀半径。
        Node(package="v550_ego_bridge", executable="trajectory_follower.py", output="screen",  # 根据真实位姿发布底盘速度。
             parameters=[os.path.join(bridge, "config", "mecanum.yaml"), {"use_sim_time": False}]),  # 保留麦轮限速、终点闭环及碰撞检查。
    ])  # 子节点等待有效定位和传感器数据后才允许跟踪。
