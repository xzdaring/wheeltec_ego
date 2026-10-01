// 智能指针：用于管理 TF Buffer、TransformListener 和 ROS 2 节点的生命周期。
#include <memory>
// std::string：保存可由参数配置的话题名和目标坐标系名。
#include <string>

// EGO Planner 当前接收 PoseStamped 类型的 /current_pose。
#include "geometry_msgs/msg/pose_stamped.hpp"
// V550 Gazebo 和实车底盘都通过 Odometry 提供里程计位姿。
#include "nav_msgs/msg/odometry.hpp"
// ROS 2 C++ 节点、发布器、订阅器、参数和日志接口。
#include "rclcpp/rclcpp.hpp"
// 捕获 TF 查询失败、外推失败等统一异常。
#include "tf2/exceptions.h"
// tf2::durationFromSec()，用于把秒转换为 TF 使用的等待时长。
#include "tf2/time.h"
// 注册 PoseStamped 的 TF 转换支持，使 Buffer::transform() 能直接转换消息。
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// Buffer 保存一段时间内收到的 TF，供带时间戳的坐标转换查询。
#include "tf2_ros/buffer.h"
// TransformListener 订阅 /tf 和 /tf_static，并把收到的变换写入 Buffer。
#include "tf2_ros/transform_listener.h"

// StateAdapter 的职责只有一个：
// 把 /odom_combined 中以 odom_combined 为参考系的车辆位姿，转换成 map
// 参考系下的 PoseStamped，再发布为 EGO Planner 需要的 /current_pose。
//
// 本节点只读取现有 TF，不发布 TF。这样不会与下面两个 TF 发布者冲突：
//   Gazebo/底盘：odom_combined -> base_footprint
//   SLAM Toolbox：map -> odom_combined
class StateAdapter : public rclcpp::Node
{
public:
  StateAdapter()
  // ROS 2 图中显示的节点名。日志和 ros2 node list 都会使用这个名称。
  : Node("v550_ego_state_adapter")
  {
    // 将接口名写成参数，而不是散落在代码里的固定字符串。
    // 仿真和实车话题名不同的时候，只需改 YAML，无需重新编译。
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_combined");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/current_pose");

    // EGO 当前发布的参考路径和局部轨迹都使用 map 坐标系，
    // 所以当前位姿也必须变换到 map 后才能进行距离和碰撞计算。
    target_frame_ = declare_parameter<std::string>("target_frame", "map");

    // 允许短时间等待与里程计消息时间对应的 TF 到达。
    // 时间过长会阻塞回调、增加控制延迟；0.1 秒适合作为仿真初始值。
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout", 0.1);

    // Buffer 必须使用本节点的时钟。在 use_sim_time=true 时，get_clock()
    // 使用 Gazebo 的 /clock，避免用系统时间查询仿真 TF 而产生外推错误。
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());

    // Listener 负责接收 /tf 和 /tf_static。必须保存为成员变量以保持存活；
    // 如果只创建成构造函数里的局部变量，构造结束后便不再接收 TF。
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 发布转换后的当前位置。队列深度 10 可吸收短暂调度抖动；
    // 对实时导航而言，下游始终更关心最新位姿，而不是积压的旧位姿。
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);

    // 订阅 V550 里程计。深度 20 给 30 Hz 左右的里程计留出短暂缓冲。
    // 回调只做一次 TF 转换和发布，保持处理路径简单且延迟可控。
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&StateAdapter::odom_callback, this, std::placeholders::_1));

    // 启动时打印实际使用的接口，方便检查是否加载了正确的 YAML 参数。
    RCLCPP_INFO(
      get_logger(), "Convert %s pose to frame '%s' and publish %s",
      odom_topic_.c_str(), target_frame_.c_str(), pose_topic_.c_str());
  }

private:
  // 每收到一帧里程计调用一次。
  // SharedPtr 避免复制整条 Odometry 消息，ROS 2 将消息所有权安全传入回调。
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // frame_id 表明 msg->pose.pose 中的数值属于哪个坐标系。
    // frame_id 为空时无法确定转换起点，继续计算会得到没有意义的位姿。
    if (msg->header.frame_id.empty()) {
      // 使用节流日志，持续收到坏消息时每 2 秒最多打印一次，
      // 避免高频里程计把终端和磁盘日志淹没。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignore odometry with an empty frame_id");
      return;
    }

    // TF 接口直接支持 PoseStamped，而 Odometry 的位姿嵌套在 pose.pose 中，
    // 因此先提取为 PoseStamped。这里保留原始 header，特别是时间戳。
    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;

    try {
      // transform() 根据 odom_pose.header.frame_id 和 header.stamp 自动查询：
      //   target_frame_ <- odom_pose.header.frame_id
      // 对当前仿真就是 map <- odom_combined。
      //
      // 必须做真实的旋转和平移变换，不能只把 frame_id 字符串改成 "map"；
      // 只改名称会让数值仍在 odom 坐标系，却被下游误认为位于 map 坐标系。
      const auto map_pose = tf_buffer_->transform(
        odom_pose, target_frame_, tf2::durationFromSec(transform_timeout_sec_));

      // 只有 TF 转换成功才发布。这样 EGO 不会收到坐标系来源不明的位姿。
      pose_pub_->publish(map_pose);
    } catch (const tf2::TransformException & error) {
      // 启动初期 SLAM 可能尚未发布 map -> odom_combined，或者 TF 到达稍晚；
      // 这些属于可恢复错误，因此记录节流告警并丢弃当前帧，不让节点退出。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform odometry from '%s' to '%s': %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), error.what());
    }
  }

  // 参数的本地副本。构造时读取一次，回调中无需重复访问参数服务器。
  std::string odom_topic_;
  std::string pose_topic_;
  std::string target_frame_;
  double transform_timeout_sec_;

  // ROS 通信对象必须保存为成员变量，否则离开构造函数后会被析构，
  // 对应的话题订阅、发布或 TF 监听也会随之停止。
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  // 初始化 ROS 2，解析 --ros-args、参数覆盖和 remap 等命令行内容。
  rclcpp::init(argc, argv);

  // 创建节点并进入事件循环；订阅回调将在收到 /odom_combined 时执行。
  rclcpp::spin(std::make_shared<StateAdapter>());

  // spin 因 Ctrl+C 等原因退出后，按顺序关闭 ROS 2 通信资源。
  rclcpp::shutdown();
  return 0;
}
