#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
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
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  GoalToPath()
  : Node("goal_to_path"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    current_pose_topic_ = declare_parameter<std::string>(
      "current_pose_topic", "/current_pose");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    path_topic_ = declare_parameter<std::string>("path_topic", "/ego_global_path");
    navigate_action_ = declare_parameter<std::string>(
      "navigate_action", "/navigate_to_pose");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    path_spacing_ = declare_parameter<double>("path_spacing", 0.30);
    min_goal_distance_ = declare_parameter<double>("min_goal_distance", 0.05);
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.10);
    xy_goal_tolerance_ = declare_parameter<double>("xy_goal_tolerance", 0.10);
    yaw_goal_tolerance_ = declare_parameter<double>("yaw_goal_tolerance", 0.10);

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

    // Publish Point 的一次点击也视为目标，起点由实时定位提供。
    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, [this](geometry_msgs::msg::PointStamped::SharedPtr point) {
        auto goal = std::make_shared<geometry_msgs::msg::PoseStamped>();
        goal->header = point->header;
        goal->pose.position = point->point;
        goal->pose.orientation.w = 1.0;
        goal_callback(goal);
      });
    // TRANSIENT_LOCAL 会保存最后一条路径。即使 motion_plan 稍晚启动，也能收到
    // 最近目标对应的参考路径，作用类似 ROS 1 中 latched publisher。
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos);

    // RViz 的 Nav2 Goal 调用 NavigateToPose action，并不会发布 /goal_pose。
    // 在桥接包提供标准 action server，既兼容 RViz，又不必启动整套 Nav2 BT Navigator。
    navigate_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      navigate_action_,
      std::bind(&GoalToPath::handle_action_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GoalToPath::handle_action_cancel, this, std::placeholders::_1),
      std::bind(&GoalToPath::handle_action_accepted, this, std::placeholders::_1));

    // action 状态检查放在定时器中，尤其是取消请求必须等 cancel 回调返回后才能
    // 转入 CANCELED；在 cancel 回调内部直接切换会违反 rcl_action 状态机。
    action_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&GoalToPath::update_action_state, this));

    RCLCPP_INFO(
      get_logger(), "目标适配器已启动: topic=%s, action=%s -> %s",
      goal_topic_.c_str(), navigate_action_.c_str(), path_topic_.c_str());
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

    // action 处于执行状态时，每次位置更新都反馈剩余距离并检查是否到达。
    update_action_state();
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!have_current_pose_) {
      // 没有起点就无法生成路径。让操作者重新点目标比凭空假设原点更安全。
      RCLCPP_WARN(get_logger(), "尚未收到 %s，暂时不能生成目标路径", current_pose_topic_.c_str());
      return;
    }

    generate_and_publish_path(*msg, nullptr);
  }

  rclcpp_action::GoalResponse handle_action_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    if (!have_current_pose_) {
      RCLCPP_WARN(get_logger(), "拒绝 Nav2 Goal：尚未收到 %s", current_pose_topic_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    // RViz 配置固定为 map；兼容录包中该插件未填写 frame_id 的请求。
    // transform_goal 会明确警告，并使用配置的 target_frame，不进行隐式 TF 猜测。
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_action_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    if (active_goal_ != goal_handle) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    // 此处只记录请求；定时器会在回调返回后安全地完成状态转换。
    cancel_requested_ = true;
    RCLCPP_INFO(get_logger(), "收到 Nav2 Goal 取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_action_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    // 新目标到来时终止旧 action，防止 RViz 同时显示两个目标仍在执行。
    if (active_goal_ && active_goal_->is_active()) {
      active_goal_->abort(std::make_shared<NavigateToPose::Result>());
    }

    geometry_msgs::msg::PoseStamped goal_in_map;
    if (!generate_and_publish_path(goal_handle->get_goal()->pose, &goal_in_map)) {
      goal_handle->abort(std::make_shared<NavigateToPose::Result>());
      return;
    }

    active_goal_ = goal_handle;
    cancel_requested_ = false;
    action_goal_in_map_ = goal_in_map;
    action_start_time_ = now();
  }

  bool generate_and_publish_path(
    const geometry_msgs::msg::PoseStamped & requested_goal,
    geometry_msgs::msg::PoseStamped * transformed_goal)
  {
    geometry_msgs::msg::PoseStamped goal_in_map;
    if (!transform_goal(requested_goal, goal_in_map)) {
      return false;
    }
    if (transformed_goal != nullptr) {
      *transformed_goal = goal_in_map;
    }

    const double dx = goal_in_map.pose.position.x - latest_pose_.pose.position.x;
    const double dy = goal_in_map.pose.position.y - latest_pose_.pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (distance < min_goal_distance_) {
      RCLCPP_INFO(
        get_logger(), "目标距离当前位置仅 %.3f m，小于阈值 %.3f m，不生成平移路径",
        distance, min_goal_distance_);
      return false;
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
    return true;
  }

  void update_action_state()
  {
    if (!active_goal_ || !active_goal_->is_active()) {
      return;
    }

    if (cancel_requested_) {
      // 空路径清除 EGO 中的旧任务，再把 action 正式置为已取消。
      publish_empty_path();
      active_goal_->canceled(std::make_shared<NavigateToPose::Result>());
      active_goal_.reset();
      cancel_requested_ = false;
      RCLCPP_INFO(get_logger(), "Nav2 Goal 已取消，旧规划路径已清除");
      return;
    }

    const double dx = action_goal_in_map_.pose.position.x - latest_pose_.pose.position.x;
    const double dy = action_goal_in_map_.pose.position.y - latest_pose_.pose.position.y;
    const double distance_remaining = std::hypot(dx, dy);
    const double current_yaw = tf2::getYaw(latest_pose_.pose.orientation);
    const double goal_yaw = tf2::getYaw(action_goal_in_map_.pose.orientation);
    // atan2(sin, cos) 把角度差限制到 [-pi, pi]，避免 179° 与 -179° 被误判相差 358°。
    const double yaw_error = std::atan2(
      std::sin(goal_yaw - current_yaw), std::cos(goal_yaw - current_yaw));

    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->current_pose = latest_pose_;
    feedback->distance_remaining = static_cast<float>(distance_remaining);
    const auto elapsed_ns = (now() - action_start_time_).nanoseconds();
    feedback->navigation_time.sec = static_cast<int32_t>(elapsed_ns / 1000000000LL);
    feedback->navigation_time.nanosec = static_cast<uint32_t>(elapsed_ns % 1000000000LL);
    feedback->number_of_recoveries = 0;
    active_goal_->publish_feedback(feedback);

    if (distance_remaining <= xy_goal_tolerance_ &&
      std::abs(yaw_error) <= yaw_goal_tolerance_)
    {
      active_goal_->succeed(std::make_shared<NavigateToPose::Result>());
      active_goal_.reset();
      RCLCPP_INFO(get_logger(), "Nav2 Goal 已到达：位置和朝向均进入容差范围");
    }
  }

  void publish_empty_path()
  {
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = now();
    empty_path.header.frame_id = target_frame_;
    path_pub_->publish(empty_path);
  }

  bool transform_goal(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output)
  {
    // RViz 正常会填写 Fixed Frame。空 frame_id 无法可靠解释，因此明确拒绝。
    if (input.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "目标 frame_id 为空，按配置的 %s 解释；RViz Fixed Frame 必须一致", target_frame_.c_str());
      output = input;
      output.header.frame_id = target_frame_;
      return true;
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
  std::string navigate_action_;
  std::string target_frame_;
  double path_spacing_;
  double min_goal_distance_;
  double transform_timeout_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  bool have_current_pose_{false};
  bool cancel_requested_{false};
  geometry_msgs::msg::PoseStamped latest_pose_;
  geometry_msgs::msg::PoseStamped action_goal_in_map_;
  rclcpp::Time action_start_time_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_server_;
  std::shared_ptr<GoalHandleNavigateToPose> active_goal_;
  rclcpp::TimerBase::SharedPtr action_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalToPath>());
  rclcpp::shutdown();
  return 0;
}
