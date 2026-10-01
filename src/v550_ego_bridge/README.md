# v550_ego_bridge

V550_mec 仿真/实车接口与 Ego Planner 2D 之间的适配包。

当前有两个适配节点：

- `state_adapter`：`/odom_combined` + TF -> `/current_pose`
- `scan_to_obstacles`：`/scan` + TF -> `/ego_obstacles`
- `goal_to_path`：`/current_pose` + `/goal_pose` -> `/ego_global_path`

两路数据都转换到 `map` 坐标系后再交给 Ego Planner。路径、机器人位置和
障碍物只有处在同一个坐标系中，距离计算和碰撞检测才有物理意义。

## 构建

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select v550_ego_bridge ego_planner --symlink-install
source install/setup.bash
```

`--symlink-install` 会让 launch 和 yaml 的源码修改直接反映到安装空间，调参数时
通常不用重新编译；C++、CMakeLists.txt 或 package.xml 改动后仍需重新构建。

## 启动

只调试状态适配器：

```bash
ros2 launch v550_ego_bridge state_adapter.launch.py
```

完整启动状态和感知适配器：

```bash
ros2 launch v550_ego_bridge bridge.launch.py
```

另开终端启动规划器：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ego_planner motion_plan
```

## 验证顺序

```bash
# 1. 确认输入存在，频率不是 0
ros2 topic hz /odom_combined
ros2 topic hz /scan

# 2. 确认 map 到传感器的整条 TF 链连通
ros2 run tf2_ros tf2_echo map laser

# 3. 确认两个适配结果正在发布
ros2 topic hz /current_pose
ros2 topic hz /ego_obstacles

# 在 RViz 使用 2D Goal Pose 后检查路径；它是事件消息，不适合用 hz
ros2 topic echo /ego_global_path --once

# 4. 确认规划器确实订阅了障碍物输出
ros2 topic info /ego_obstacles --verbose
```

若实际雷达帧不叫 `laser`，把第三条命令中的名称换成
`ros2 topic echo /scan --once --field header.frame_id` 得到的值。

只有对应 TF 可用时适配器才发布转换后的数据。两个适配器都只读取 TF，不发布
TF，因此不会与 SLAM Toolbox 或 Gazebo 的 TF 发布者争夺同一坐标关系。

`config/scan_to_obstacles.yaml` 中最先需要理解的参数是：

- `min_range`：剔除盲区和车体自身反射。
- `max_range`：限制局部规划关注距离。
- `beam_step`：每隔多少束取一点；值越小障碍更密，计算量也越大。
- `transform_timeout`：等待采样时刻 TF 的最长秒数。

`config/goal_to_path.yaml` 中：

- `path_spacing`：直线参考路径的点间距。
- `min_goal_distance`：小于该距离时认为无需生成平移路径。

当前 `goal_to_path` 生成的是直线全局参考路径，适合先验证完整接口链和开阔场景。
它不是全局绕障算法；以后遇到墙体完全截断直线路径时，应把它替换为 Nav2 全局
规划器输出的 `nav_msgs/Path`，同时继续发布到 `/ego_global_path`。
