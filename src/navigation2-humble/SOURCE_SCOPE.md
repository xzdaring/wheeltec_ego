# EGO工程中的Nav2源码范围

2026-10-07，仅精简电脑工作空间，保留nav2_common、nav2_msgs、nav2_util、nav2_lifecycle_manager、nav2_rviz_plugins，共5个源码包。

EGO目标适配器依赖NavigateToPose消息，RViz依赖Nav2 Goal插件；插件还依赖util和lifecycle_manager，因此一并保留。其余32个包的源码及对应build/install目录已移出工程。系统已安装这些包，保留的厂家导航入口仍可解析系统依赖；本次不卸载系统ROS软件。

nav2_bringup由/opt/ros/humble提供。重新部署环境时，仍须安装manifest中的依赖，不能将源码精简理解为完全移除了Nav2运行库。

备份：/ros_workspace/_source_backups/nav2_trim_20261007_155849
manifest.json记录移除包和原工作空间；src保存移出的顶层源码目录，build/install保存对应原构建目录。

验证：ego_planner、v550_navigation、v550_ego_bridge构建通过；ego_sim.launch.py和ego_real.launch.py的--show-args均通过。未启动仿真或实车运动，未提交或推送Git。
