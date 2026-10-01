# 构建验证记录

日期：2026-10-01

源码工作区：`/ros_workspace/wheeltec_sim/wheeltec_nav-main`

环境：Ubuntu 22.04、ROS 2 Humble、x86_64。

## 结果

执行 `./scripts/build_workspace.sh` 完成全工作区构建：

```text
Summary: 57 packages finished [9.41s]
```

此结果是首次全量编译和依赖修复后的最终增量复编，0 失败、0 跳过。首次全量编译约 11 分钟。原有第三方源码存在编译警告，未将无关代码整体重写。使用 `BUILD_TESTING=OFF`，没有执行单元测试、Gazebo 运行测试或实车测试。

## 修复与依赖

- 原工作区已有普通安装缓存，保持普通安装方式；显式设置 `AMENT_CMAKE_SYMLINK_INSTALL=OFF`。
- 补入 `src/depend/serial_ros2`，来自 `/ros_workspace/wheeltec_nav/src/depend/serial_ros2`，解决底盘包找不到 serial。
- 用户安装了系统 `libpcap-dev` / `libpcap0.8-dev` 1.10.1-4ubuntu1.22.04.2，解决雷达驱动缺少 pcap.h。曾用本地解压依赖单独验证，最终清除雷达包 CMake 缓存后用系统依赖重编通过；本地依赖不上传。
- 最终雷达可执行文件的 libpcap 动态链接指向系统库。
- `scripts/build_workspace.sh` 限制构建并行度，以控制内存使用。

## 源码发布范围

包含源码、launch、参数、模型网格和所需资源；不包含 build/install/log、源码内生成的共享库、录包或本地编辑器配置。保留目标仓库原始提交历史。

## 运行限制

编译通过不证明运行依赖齐全。检查环境中未发现系统 Gazebo ROS 包，因此未运行仿真；启动仿真前需要确认 gazebo_ros、gazebo_plugins、slam_toolbox 等运行依赖。

`v550_navigation/launch/real_navigation.launch.py` 引用但尚未提供 `nav2_v550_real.yaml`。这不影响 CMake 编译，但实车启动前必须补齐并验证。

当前仓库是 V550 仿真/导航基线，尚未集成新二维 EGO 工程。
