#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/**
 * @brief 把 RViz 的单个目标位姿转换成 EGO Planner 所需的全局参考路径。
 *
 * EGO Planner 的输入不是一个终点，而是一串参考路径点。因此本节点使用机器人
 * 当前位姿和目标位姿生成等间距直线路径。直线只是第一版全局引导；以后接入
 * Nav2 或其他全局规划器时，只需替换本节点，EGO Planner 的接口保持不变。
 */
class GoalToPath : public rclcpp::Node
{
public:
  GoalToPath()
  : Node("goal_to_path"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    current_pose_topic_ = declare_parameter<std::string>(
      "current_pose_topic", "/current_pose");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    path_topic_ = declare_parameter<std::string>("path_topic", "/ego_global_path");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    path_spacing_ = declare_parameter<double>("path_spacing", 0.30);
    min_goal_distance_ = declare_parameter<double>("min_goal_distance", 0.05);
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.10);

    if (path_spacing_ <= 0.0) {
      throw std::invalid_argument("path_spacing 必须大于 0");
    }
    if (min_goal_distance_ < 0.0) {
      throw std::invalid_argument("min_goal_distance 不能小于 0");
    }
    transform_timeout_ = std::max(0.0, transform_timeout_);

    current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      current_pose_topic_, 10,
      std::bind(&GoalToPath::current_pose_callback, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      std::bind(&GoalToPath::goal_callback, this, std::placeholders::_1));

    // TRANSIENT_LOCAL 会保存最后一条路径。即使 motion_plan 稍晚启动，也能收到
    // 最近目标对应的参考路径，作用类似 ROS 1 中 latched publisher。
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos);

    RCLCPP_INFO(
      get_logger(), "目标适配器已启动: %s + %s -> %s",
      current_pose_topic_.c_str(), goal_topic_.c_str(), path_topic_.c_str());
  }

private:
  void current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // state_adapter 应当已经输出 map 位姿。这里仍检查坐标系，尽早暴露接线错误。
    if (msg->header.frame_id != target_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "忽略当前位置：期望 frame_id=%s，实际为 '%s'",
        target_frame_.c_str(), msg->header.frame_id.c_str());
      return;
    }

    latest_pose_ = *msg;
    have_current_pose_ = true;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!have_current_pose_) {
      // 没有起点就无法生成路径。让操作者重新点目标比凭空假设原点更安全。
      RCLCPP_WARN(get_logger(), "尚未收到 %s，暂时不能生成目标路径", current_pose_topic_.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped goal_in_map;
    if (!transform_goal(*msg, goal_in_map)) {
      return;
    }

    const double dx = goal_in_map.pose.position.x - latest_pose_.pose.position.x;
    const double dy = goal_in_map.pose.position.y - latest_pose_.pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (distance < min_goal_distance_) {
      RCLCPP_INFO(
        get_logger(), "目标距离当前位置仅 %.3f m，小于阈值 %.3f m，不生成平移路径",
        distance, min_goal_distance_);
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = target_frame_;

    // 至少分成两段，从而产生三个路径点。EGO 优化器要求输入点数不少于 3。
    const int segment_count = std::max(2, static_cast<int>(std::ceil(distance / path_spacing_)));
    const double travel_yaw = std::atan2(dy, dx);
    tf2::Quaternion travel_orientation;
    travel_orientation.setRPY(0.0, 0.0, travel_yaw);

    path.poses.reserve(static_cast<std::size_t>(segment_count + 1));
    for (int index = 0; index <= segment_count; ++index) {
      const double ratio = static_cast<double>(index) / segment_count;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = latest_pose_.pose.position.x + ratio * dx;
      pose.pose.position.y = latest_pose_.pose.position.y + ratio * dy;
      pose.pose.position.z = 0.0;

      if (index == segment_count) {
        // 最后一点使用用户在 RViz 中画出的朝向，后续控制器靠它完成终点转向。
        pose.pose.orientation = goal_in_map.pose.orientation;
      } else {
        // 中间点朝向沿路径切线，使路径的几何方向清楚且四元数始终有效。
        pose.pose.orientation = tf2::toMsg(travel_orientation);
      }
      path.poses.push_back(pose);
    }

    path_pub_->publish(path);
    RCLCPP_INFO(
      get_logger(), "已生成参考路径: 起点(%.2f, %.2f), 终点(%.2f, %.2f), %zu 个点",
      latest_pose_.pose.position.x, latest_pose_.pose.position.y,
      goal_in_map.pose.position.x, goal_in_map.pose.position.y, path.poses.size());
  }

  bool transform_goal(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output)
  {
    // RViz 正常会填写 Fixed Frame。空 frame_id 无法可靠解释，因此明确拒绝。
    if (input.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "忽略目标：/goal_pose 的 frame_id 为空");
      return false;
    }

    if (input.header.frame_id == target_frame_) {
      output = input;
      return true;
    }

    try {
      // 使用目标消息的时间查询 TF；stamp=0 时 tf2 会取最新可用变换。
      output = tf_buffer_.transform(
        input, target_frame_, tf2::durationFromSec(transform_timeout_));
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(
        get_logger(), "目标无法从 %s 转换到 %s: %s",
        input.header.frame_id.c_str(), target_frame_.c_str(), error.what());
      return false;
    }
  }

  std::string current_pose_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string target_frame_;
  double path_spacing_;
  double min_goal_distance_;
  double transform_timeout_;

  bool have_current_pose_{false};
  geometry_msgs::msg::PoseStamped latest_pose_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalToPath>());
  rclcpp::shutdown();
  return 0;
}
