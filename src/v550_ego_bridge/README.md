# v550_ego_bridge

V550_mec 仿真/实车接口与 Ego Planner 2D 之间的适配包。

当前有两个适配节点：

- `state_adapter`：`/odom_combined` + TF -> `/current_pose`
- `scan_to_obstacles`：`/scan` + TF -> `/ego_obstacles`
- `goal_to_path`：`/current_pose` + `/goal_pose` -> `/ego_global_path`
  同时提供 `/navigate_to_pose` action，兼容 RViz 的 **Nav2 Goal** 工具。

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

V550 仿真同时启动的 DDS participant 较多。每个终端先加载统一环境，扩大
CycloneDDS 在本机回环接口上的自动 participant index 范围：

```bash
source src/v550_ego_bridge/scripts/setup_v550_dds.bash
```

只修改 `ROS_DOMAIN_ID` 只会改变 DDS 端口基址，不会扩大索引范围；真正解决
`Failed to find a free participant index` 的是脚本设置的 `CYCLONEDDS_URI`。

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

## 2026-10-02 录包修复与统一仿真入口

先 Ctrl+C 关闭旧仿真、bridge 和 motion_plan，避免重复节点。工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/v550_ego_bridge/scripts/setup_v550_dds.bash
ros2 launch v550_ego_bridge ego_sim.launch.py
```

RViz 固定坐标系为 map。Nav2 Goal 一次拖拽指定位置和朝向；Publish Point 一次
点击也作为目标，起点来自 /current_pose。蓝色为 /ego_global_path 直线参考路径，
绿色为 /visual_local_trajectory 优化轨迹，橙色为 /visual_local_obstacles 实际
膨胀栅格（半径 0.5 m），红色为 /ego_obstacles 激光点。
膨胀显示在收到位置和雷达后即可出现，路径在收到目标后出现。

录包 rosbag_all 的 /rosout 有三次“拒绝 Nav2 Goal：目标 frame_id 为空”，
/ego_global_path 和 /visual_local_trajectory 均零条；两次 /clicked_point 触发
旧的手工拼点逻辑后出现 984 次空轨迹告警。修复短段重采样用 floor 丢失终点的
错误后，以录包首帧位置和障碍、首次点击坐标测试：一次 action 产生 5 点参考路径
与 10 点局部轨迹；目标发送前已发布 1825 个膨胀点。

兼容此 RViz 空 frame 请求时会明确报警并按 target_frame=map 解释，其他坐标系
仍使用 TF。这里没有把目标加入障碍列表。参考直线不保证绕过封闭墙体；不可行的
目标不能保证产生局部轨迹。尚未实现轨迹到 /cmd_vel 的跟踪控制，本入口用于
规划与可视化，不能据此声称小车已自动到达目标。

## 2026-10-02 V550 麦轮位姿轨迹与闭环跟踪

rosbag_all2 只有 /rosout 的6条录包器日志及空的 /events/write_split，没有轨迹、
定位、激光或速度消息。这次定位问题以当前源码和新的隔离闭环试验为依据。

数据链：Nav2 Goal -> goal_to_path -> /ego_global_path -> EGO二维位置优化
-> /visual_local_trajectory -> trajectory_follower.py 中的SE(2)运动学适配
-> /ego_trajectory（紫色朝向箭头、每个位姿带计划时间）-> /cmd_vel -> Gazebo。

EGO优化位置曲线，姿态适配层按当前位置和最终yaw的最短角差平滑生成车体朝向，
使用四轮速度/离散加速度约束安排参考时间。控制器采用几何前视和位姿反馈，
不要求严格按时间表到点，最终输出再次经过四轮速度和加速度限幅。
这不是把现有B样条优化器改造成完整SE(2)非线性动力学优化器。

对于45度X型麦轮，轮序LF/RF/LB/RB，车体系x前、y左、yaw逆时针，轮周速度：

```
u_lf = vx - vy - (L+W)*wz
u_rf = vx + vy + (L+W)*wz
u_lb = vx + vy - (L+W)*wz
u_rb = vx - vy + (L+W)*wz
轮角速度 = u / r
```

L=0.0788 m、W=0.09829 m 取自URDF轮心间距的一半；r≈0.03717 m 来自STL
轮胎外形。有效滚动半径与电机额定参数尚未在实车标定。config/mecanum.yaml
中的0.30 m/s轮周速度、0.40 m/s²轮周加速度和0.60 rad/s角速度是保守仿真值。
跟踪器保留vy，可不改变车头朝向横移；近终点分别闭环控制位置和姿态，允许纯旋转。
旧代码的单位起始速度/加速度已去掉，起始平移速度由里程计转换到map提供。

碰撞采用半径0.20m的整车包络圆，不因转向漏检车角；短时预测碰撞、定位/雷达
超时、局部路径超时、取消或规划失败均停车。正常运动限制轮加速度，紧急零速
不经过缓慢减速。任务stamp过滤旧目标路径，正常退出先发零速度。
这是运动学仿真，现有gazebo_ros_planar_move仍驱动整车，四轮为固定关节；没有
模拟麦轮滚子接触或打滑。/ego_wheel_reference仅显示计算轮速，不冒充实际编码器。

先停止旧启动实例，避免重复 /cmd_vel 发布者。启动命令不变：

```bash
cd /ros_workspace/wheeltec_sim/wheeltec_nav-main
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/v550_ego_bridge/scripts/setup_v550_dds.bash
ros2 launch v550_ego_bridge ego_sim.launch.py
```

本入口现在自动启动跟踪器，点击可行目标后仿真车会实际运动。Nav2保持关闭，
不要同时启动其他速度控制器。选择距障碍足够远的短目标，先验证横移，再验证
终点朝向；RViz紫色箭头是车体朝向，绿色是EGO位置曲线，二者无需方向相同。

可复现测试（使用独立Domain，切勿与实车或另一仿真共用）：

```bash
python3 src/v550_ego_bridge/test/test_model.py
# 在新的测试终端中加载上面的环境后覆盖Domain：
export ROS_DOMAIN_ID=94
python3 src/v550_ego_bridge/test/test_closed_loop.py
# 真正Gazebo测试使用独立master端口，脚本启动自己的无界面实例并清理：
export ROS_DOMAIN_ID=95
export GAZEBO_MASTER_URI=http://127.0.0.1:11365
python3 src/v550_ego_bridge/test/test_gazebo.py
```

录包请另开终端执行与启动端相同的三条source（包括setup_v550_dds.bash），先
`ros2 topic list`确认存在/current_pose、/ego_global_path、/ego_trajectory和/cmd_vel，
再运行`ros2 bag record -a -o /ros_workspace/rosbag_all4`。录完用
`ros2 bag info /ros_workspace/rosbag_all4`确认这些话题有消息。

### 上一版验收记录（本次修复前）

独立底盘闭环：横移位置误差0.02998m、平移加90度朝向误差0.03999rad、
纯旋转位置不变且朝向误差0.04000rad；取消和雷达失联输出零速度。
四轮轮周速度峰值0.30m/s，横向速度峰值约0.188m/s。
真实Gazebo无界面入口测试：目标相对起点(+0.30m,+0.15m)、yaw=0.5rad，
返回NavigateToPose SUCCEEDED，位置误差0.02982m、yaw误差0.00955rad，
发布15条位姿轨迹，并在到达后输出零速度。测试使用独立Domain95及Gazebo端口，
结束后已关闭测试实例。上述是此场景验收结果，不代表任意障碍布局均可达。

## rosbag_all3：绕障、前视行驶和膨胀一致性修复

本包确有完整数据：/cmd_vel 4645条，其中556条vx<-0.01m/s；/scan视场
约[-100°,100°]。原参考路径是起终点直线，原跟踪朝向逐渐对齐最终yaw，
二者分别解释了穿障参考和倒行。旧激光点云每帧替换，不保留视场外墙面。

现在链路为：目标请求/ego_reference_request -> global_route.py ->
基于/map和/ego_obstacles的膨胀Dijkstra参考/ego_global_path -> EGO优化 ->
全段硬碰撞检查 -> 跟踪器再次检查 -> /cmd_vel。EGO数值解不可行时，只可回退
到同样检查通过的绕障参考折线，不能回退到任意直线。

/ego_costmap是唯一权威执行边界，RViz已添加该显示：100代表实体包络或未知，
1..99为安全余量，0为可行驶。实体包络0.20m，膨胀总半径0.25m，另考虑障碍
方格的半对角线；未知格禁止通行，但不把未知当实体墙向自由区膨胀。
这不保证车体周边所有区域已经观测，首次测试仍需在已知安全的开阔区域启动。规划和控制都不允许进入100格。
起点若只是进入1..99安全余量区，允许代价单调不增加的路线退出；若已触及
实体碰撞边界，则停车并输出NO_SAFE_ROUTE，需要重新定位或人工移到安全处。

默认行驶先对准路线方向，保留小幅横移纠偏，vx不指令倒车；位置到达后再对齐
目标yaw。紫色姿态箭头也采用这一策略。这样不会为了最终朝向而倒着穿行。
地图、雷达或统一安全地图超时停止；历史墙面由SLAM地图保留，解决后视盲区
里已知墙体被单帧雷达遗忘的问题。当前地图未知的后方区域仍不允许盲行。

启动命令仍为ros2 launch v550_ego_bridge ego_sim.launch.py（先source统一DDS脚本），
务必停止旧实例。查看不可行原因：ros2 topic echo /ego_planning_status。
本次改动覆盖上一节“可保持车头朝向完成整段横移”的默认行为：前视模式优先
让行驶方向落入雷达视场，麦轮vy仍用于纠偏，没有改成差速底盘。

### 本轮验证与复现（2026-10-02）

- 编译通过：ego_planner、v550_ego_bridge、v550_navigation。
- 栅格回归：绕墙、封闭通道拒绝、膨胀余量单调退出、float32坐标转换安全通过。
- 独立底盘绕墙：目标(0.9,0,0)，位置误差0.02396m，yaw误差0.03723rad；
  测试逐条断言vx不倒车，并独立计算圆形车体到矩形墙距离，确认没有碰撞。
- 完整Gazebo+SLAM：目标相对起点(+0.30,+0.15)、yaw=0.5，Action返回成功，
  位置误差0.02959m，yaw误差0.000481rad，到达后零速度。

前置雷达不会观测自身车身区域，因此SLAM起点有时为未知。global_route仅在
定位新鲜时豁免当前车体内的未知格，不清除实测障碍及其膨胀。未知边界没有
实体膨胀，所以安全保证是针对已观测障碍；实际试车仍需先确认周边安全。

规划分工：Dijkstra提供绕障参考，EGO尝试平滑局部路径；当优化失败或切角时，
绿色局部路径可能是验证通过的参考折线，不代表每次都成功生成B样条。
跟踪器仍使用V550麦轮逆运动学和四轮速度/加速度约束，拐角允许先转向。
禁止仅凭RViz中的绿色线判断成功：还要看Action结果、位置误差和是否零速停止。

在独立测试终端中（先source ROS、install及setup_v550_dds.bash）：

```bash
python3 src/v550_ego_bridge/test/test_grid.py
export ROS_DOMAIN_ID=97
python3 src/v550_ego_bridge/test/test_wall.py
export ROS_DOMAIN_ID=96
python3 src/v550_ego_bridge/test/test_closed_loop.py
export ROS_DOMAIN_ID=98
export GAZEBO_MASTER_URI=http://127.0.0.1:11365
python3 src/v550_ego_bridge/test/test_gazebo.py
```

这些测试自行启动节点；不要和正常仿真共用Domain或Gazebo master端口。
已接触实体包络、通路完全封闭或目标位于膨胀区域时，预期结果是停车并报告
NO_SAFE_ROUTE；不通过缩小膨胀半径强行通行。当前action保持等待，可在RViz取消。
