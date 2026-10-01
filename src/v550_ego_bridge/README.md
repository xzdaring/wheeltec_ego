# v550_ego_bridge

V550_mec 仿真/实车接口与 Ego Planner 2D 之间的适配包。

当前实现 `state_adapter`：订阅 `/odom_combined`，根据消息时间查询 TF，
把位姿从里程计坐标系转换到 `map`，再发布 `/current_pose`。

```bash
ros2 launch v550_ego_bridge state_adapter.launch.py
```

只有当 `map -> odom_combined` TF 可用时才会发布位姿。适配节点不发布 TF，
因此不会与 SLAM Toolbox 或 Gazebo 的 TF 发布者冲突。
