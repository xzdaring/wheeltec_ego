# wheeltec_ego

在轮趣科技 V550_mec 上部署 EGO Planner 的开发仓库。

本次导入基线来自 `/ros_workspace/wheeltec_sim/wheeltec_nav-main`：包含 V550 Gazebo 仿真、底盘和雷达源码、SLAM 配置及 Nav2 Humble 源码。二维 EGO 工程尚未集成到本仓库，不要把此基线当作已经完成 EGO 导航。

## 编译

Ubuntu 22.04 / ROS 2 Humble。在新终端只加载系统 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
cd /path/to/wheeltec_ego
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
  --parallel-workers 2 --continue-on-error \
  --cmake-args -DBUILD_TESTING=OFF -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.bash
```

也可以在完成依赖安装后执行 `./scripts/build_workspace.sh`，使用相同构建参数。

首次使用 rosdep 需要按系统情况执行 `sudo rosdep init` 和 `rosdep update`。雷达驱动需要 `libpcap-dev`；底盘所需 `serial` 源码已放入 `src/depend/serial_ros2`，来自本机已使用的 wheeltec_nav 工作区。所有第三方代码保留原许可证和作者声明。

当前采用普通安装方式；已有普通安装缓存时，不直接加 `--symlink-install`，否则可能出现 Python 包目录与符号链接冲突。改变方式应使用独立构建目录或备份并重建缓存。

## 仿真

构建成功且 Gazebo/SLAM 运行依赖安装后：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch v550_navigation simulation_nav.launch.py start_nav2:=false start_slam:=true
```

默认模型使用平面运动插件，不是四轮滚子接触动力学模型。后续接入 EGO 时保持旧 Nav2 速度输出关闭。

## 维护

只提交源码、配置、地图/模型等必要资源和文档。`.gitignore` 排除 `build/`、`install/`、`log/`、源码目录内生成的 `.so` 等产物，以及录包和本地编辑器设置。

```bash
git status --short
git diff
git add src README.md .gitignore BUILD_STATUS.md
git diff --cached --stat
git commit -m "Describe the actual change"
git push origin main
```

具体编译结果见 `BUILD_STATUS.md`。编译通过不等于 Gazebo 或实车导航验收通过。

本基线的 `real_navigation.launch.py` 引用的 `nav2_v550_real.yaml` 尚缺失；实车启动前需要补齐并单独验证，不要直接用仿真配置替代实车限速配置。
