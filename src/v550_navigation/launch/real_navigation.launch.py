import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    V550 实车自主导航启动文件。

    启动内容：
    1. 真实底盘、EKF、URDF 和 IMU
    2. 真实 M10_P 雷达
    3. /scan -> /scan_slam 归一化节点
    4. slam_toolbox 在线建图
    5. Nav2 navigation_launch.py
    6. RViz

    不启动：
    1. AMCL
    2. map_server
    3. 预加载静态地图
    """

    pkg = get_package_share_directory('v550_navigation')

    base_pkg = get_package_share_directory('turn_on_wheeltec_robot')

    nav2_pkg = get_package_share_directory('nav2_bringup')

    # launch参数

    # 是否启动真实底盘和雷达
    # 如果底盘已经由其他终端启动，则使用 bringup_hardware:=false
    bringup_hardware = LaunchConfiguration('bringup_hardware')

    # 是否启动 RViz
    use_rviz = LaunchConfiguration('use_rviz')

    # 是否启动在线 SLAM
    start_slam = LaunchConfiguration('start_slam')

    # 是否启动 Nav2
    start_nav2 = LaunchConfiguration('start_nav2')

    # 文件路径
    # SLAM 参数文件
    slam_params = os.path.join(
        pkg,
        'config',
        'slam_v550.yaml',
    )
    # 实车 Nav2 参数文件
    # 必须使用安全速度版本
    nav2_params = os.path.join(
        pkg,
        'config',
        'nav2_v550_real.yaml',
    )

    # RViz 配置
    rviz_config = os.path.join(
        pkg,
        'rviz',
        'v550.rviz',
    )


    # 真实底盘、EKF、URDF 和 IMU

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                base_pkg,
                'launch',
                'turn_on_wheeltec_robot.launch.py',
            )
        ),
        condition = IfCondition(bringup_hardware),
    )

    # 真实M10_P雷达
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                base_pkg,
                'launch',
                'wheeltec_lidar.launch.py',
            )
        ),
        # 只有 bringup_hardware:=true 时才启动
        condition=IfCondition(bringup_hardware),
    )

    # 雷达扫面归一化
    normalizer = Node(
        package='v550_navigation',
        executable='normalize_scan',
        name='v550_scan_normalizer',
        parameters=[{
            # 原始雷达话题
            'input_topic': '/scan',

            # SLAM 使用的话题
            'output_topic': '/scan_slam',

            # 固定输出点数
            'scan_points': 800,

            # 前方约 200°视场
            'fov_min': -1.7453292519943295,
            'fov_max': 1.7453292519943295,

            # 实车不使用 Gazebo 仿真时间
            'use_sim_time': False,
        }],
        # 只有启动 SLAM 时才需要归一化
        condition=IfCondition(start_slam),
        output='screen',
    )
    # slam_toolbox在线建图

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            # 公共 SLAM 配置
            slam_params,

            # 覆盖仿真配置中的 /scan
            # 实车 SLAM 使用固定点数 /scan_slam
            {
                'scan_topic': '/scan_slam',
                'use_sim_time': False,
            },
        ],
        condition=IfCondition(start_slam),
        output='screen',
    )

    # nav2导航线

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_pkg,
                'launch',
                'navigation_launch.py',
            )
        ),
        launch_arguments={
            # 实车不使用 Gazebo 时间
            'use_sim_time': 'false',

            # 实车低速度 Nav2 参数
            'params_file': nav2_params,

            # 生命周期节点自动激活
            'autostart': 'true',

            # 实车建议关闭组件容器，方便查看每个节点日志
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(start_nav2),
    )

    # Rviz

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',

        # 自动加载 Map、LaserScan、TF、RobotModel、
        # Costmap、Path 和 Nav2 Goal 配置
        arguments=['-d', rviz_config],

        # RViz 使用系统时间
        parameters=[{'use_sim_time': False}],

        condition=IfCondition(use_rviz),
        output='screen',
    )

    # launch描述
    return LaunchDescription([  # launch入口必须返回动作列表，不能返回参数替换对象。
        # 默认同时启动底盘和雷达
        DeclareLaunchArgument(
            'bringup_hardware',
            default_value='true',
            description='启动真实底盘、EKF 和 M10_P 雷达',
        ),

        # 默认启动 RViz
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动 RViz2',
        ),

        # 默认启动在线 SLAM
        DeclareLaunchArgument(
            'start_slam',
            default_value='true',
            description='是否启动 slam_toolbox',
        ),

        # 默认启动 Nav2
        DeclareLaunchArgument(
            'start_nav2',
            default_value='true',
            description='是否启动 Nav2 navigation_launch.py',
        ),
        # 继承终端DDS设置，避免此子launch强制切换RMW或localhost而与EGO节点失联。

        # 启动顺序：
        # 1. 底盘和雷达
        # 2. 扫描归一化
        # 3. 在线 SLAM
        # 4. Nav2
        # 5. RViz
        hardware,
        lidar,
        normalizer,
        slam,
        nav2,
        rviz,
    ])
 