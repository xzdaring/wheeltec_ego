import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import Command,LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg = get_package_share_directory('v550_navigation')

    gazebo_ros = get_package_share_directory('gazebo_ros')

    nav2_bringup = get_package_share_directory('nav2_bringup')

    urdf_share = get_package_share_directory('wheeltec_robot_urdf')

     # 仿真模型、世界、SLAM、Nav2 配置路径
    urdf = os.path.join(pkg, 'urdf', 'v550_mec_sim_kinematic.urdf')
    world = os.path.join(pkg, 'worlds', 'v550_room.world')
    slam_params = os.path.join(pkg, 'config', 'slam_v550.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_v550.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'v550.rviz')

    # launch传参数 

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('use_gui')
    start_slam = LaunchConfiguration('start_slam')
    start_nav2 = LaunchConfiguration('start_nav2')

    # 使用 xacro 命令读取当前唯一的 V550_mec 仿真 URDF。
    # ParameterValue 保证 robot_description 按字符串传给节点。
    robot_description = ParameterValue(
        Command(['xacro ',urdf]),
        value_type=str,
    )

    # Gazebo 使用 package://wheeltec_robot_urdf/... 网格时，
    # spawn_entity.py 会转换 model:// 路径。
    # 因此需要将 wheeltec_robot_urdf 的父目录加入 GAZEBO_MODEL_PATH
    gazebo_model_path = os.path.dirname(urdf_share)
    old_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if old_model_path:
        gazebo_model_path = gazebo_model_path + ':' + old_model_path

    return LaunchDescription([
        # 仿真使用Gazebo
        DeclareLaunchArgument('use_sim_time',default_value='true'),

        # 是否打开rviz
        DeclareLaunchArgument('use_rviz',default_value='true'),

         # 是否打开 Gazebo 图形界面
        DeclareLaunchArgument('use_gui', default_value='true'),

        # 是否启动 SLAM Toolbox
        DeclareLaunchArgument('start_slam', default_value='true'),

        # 是否启动 Nav2
        DeclareLaunchArgument('start_nav2', default_value='true'),

        # 确保 Gazebo 能解析 package_to_model 后的网格路径
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        # 避免当前系统中的 CycloneDDS 与 Gazebo 插件服务发现冲突
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        ),

        # 本机仿真只和本机通信，避免同网段其他 ROS 节点混入
        SetEnvironmentVariable(
            'ROS_LOCALHOST_ONLY',
            '1',
        ),

        # 启动Gazebo 服务端
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros,'launch','gzserver.launch.py')
            ),
            launch_arguments={
                'world':world,
                'verbose':'true',
            }.items(),
        ),

        # 启动Gazebo客户端，只负责三维显示
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros,'launch','gzclient.launch.py')
            ),
            condition = IfCondition(use_gui),
        ),

        # 发布robot_description,固定关节和动态关节 TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output ='screen',
        ),

        # 发布 base_footprint -> base_link 静态 TF
        # Gazebo 物理根节点保留为有质量的 base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=[
                '0', '0', '0.02027',
                '0', '0', '0',
                'base_footprint', 'base_link',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 发布四个车轮joint 的状态

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        # 从 /robot_description 话题读取 URDF，并将其生成到 Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'v550_mec',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.08',
                '-package_to_model',

                 # Gazebo 服务未就绪时最多等待 60 秒
                '-timeout', '60',
            ],
            output='screen',
        ),

        # 在线建图
        # 输入 /scan 和 odom_combined -> base_footprint
        # 输出 /map 和 map -> odom_combined
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_params,
                {'use_sim_time': use_sim_time},
            ],
            condition=IfCondition(start_slam),
            output='screen',
        ),

        # 启动Nav2导航部分
        # navigation_launch.py 不启动 AMCL map_server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_bringup,
                    'launch',
                    'navigation_launch.py',
                )
            ),
            launch_arguments={
                'use_sim_time':use_sim_time,
                'params_file':nav2_params,
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
            condition = IfCondition(start_nav2),
        ),

        # RViz 使用空配置启动，后续手动配置 Map、LaserScan、TF 和 Nav2 Goal
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen',
        ),

    ])
