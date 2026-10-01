#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class StateAdapter : public rclcpp::Node
{
public:
  StateAdapter()
  : Node("v550_ego_state_adapter")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_combined");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/current_pose");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout", 0.1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&StateAdapter::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Convert %s pose to frame '%s' and publish %s",
      odom_topic_.c_str(), target_frame_.c_str(), pose_topic_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignore odometry with an empty frame_id");
      return;
    }

    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;

    try {
      const auto map_pose = tf_buffer_->transform(
        odom_pose, target_frame_, tf2::durationFromSec(transform_timeout_sec_));
      pose_pub_->publish(map_pose);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform odometry from '%s' to '%s': %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), error.what());
    }
  }

  std::string odom_topic_;
  std::string pose_topic_;
  std::string target_frame_;
  double transform_timeout_sec_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateAdapter>());
  rclcpp::shutdown();
  return 0;
}
