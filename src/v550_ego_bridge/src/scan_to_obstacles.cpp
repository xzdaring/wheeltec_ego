#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/**
 * @brief 把 V550 的二维激光扫描转换为 EGO Planner 可读取的障碍物点云。
 *
 * 数据链路：/scan (laser 坐标系) -> TF 变换 -> /ego_obstacles (map 坐标系)。
 * 单独建立适配器的原因是：底盘驱动只负责提供标准 LaserScan，规划器只负责
 * 使用统一坐标系下的障碍点；两边不需要了解对方的内部实现，之后上实车时只需
 * 修改参数，而不必改规划算法。
 */
class ScanToObstacles : public rclcpp::Node
{
public:
  ScanToObstacles()
  : Node("scan_to_obstacles"),
    // Buffer 保存一段时间内的 TF。收到一帧 scan 后，可按该帧时间查询坐标关系。
    tf_buffer_(this->get_clock()),
    // TransformListener 订阅 /tf 和 /tf_static，并自动把变换写入 tf_buffer_。
    tf_listener_(tf_buffer_)
  {
    // 参数而不是硬编码话题名，便于仿真和实车使用不同的话题而无需重新编译。
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    obstacles_topic_ = this->declare_parameter<std::string>(
      "obstacles_topic", "/ego_obstacles");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
    min_range_ = this->declare_parameter<double>("min_range", 0.25);
    max_range_ = this->declare_parameter<double>("max_range", 12.0);
    transform_timeout_ = this->declare_parameter<double>("transform_timeout", 0.10);
    beam_step_ = this->declare_parameter<int>("beam_step", 4);

    // 防止错误参数导致除零、负超时或把所有激光点过滤掉。
    beam_step_ = std::max(1, beam_step_);
    transform_timeout_ = std::max(0.0, transform_timeout_);
    if (min_range_ < 0.0 || max_range_ <= min_range_) {
      throw std::invalid_argument("参数必须满足 0 <= min_range < max_range");
    }

    // 激光传感器通常用 BEST_EFFORT QoS；SensorDataQoS 能与 Gazebo 和多数实车雷达兼容。
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanToObstacles::scan_callback, this, std::placeholders::_1));

    // 障碍物也是高频传感器数据。队列保持较小，可避免规划器处理已经过时的环境。
    obstacles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      obstacles_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      this->get_logger(),
      "感知适配器已启动: %s -> %s, 输出坐标系=%s, 每 %d 束取一点",
      scan_topic_.c_str(), obstacles_topic_.c_str(), target_frame_.c_str(), beam_step_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (scan->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "收到的 /scan 没有 frame_id，无法进行 TF 坐标变换");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      // 使用扫描消息自己的时间戳，确保障碍点与机器人在采样时刻的位姿对应。
      // 一帧只查询一次 TF；每个点重复查询会显著增加开销且可能得到不一致的变换。
      transform = tf_buffer_.lookupTransform(
        target_frame_, scan->header.frame_id, rclcpp::Time(scan->header.stamp),
        rclcpp::Duration::from_seconds(transform_timeout_));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "无法把 %s 变换到 %s，本帧激光跳过: %s",
        scan->header.frame_id.c_str(), target_frame_.c_str(), error.what());
      return;
    }

    const double effective_min = std::max(min_range_, static_cast<double>(scan->range_min));
    const double effective_max = std::min(max_range_, static_cast<double>(scan->range_max));
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve((scan->ranges.size() + beam_step_ - 1) / beam_step_);

    for (std::size_t index = 0; index < scan->ranges.size(); index += beam_step_) {
      const double range = scan->ranges[index];

      // NaN/Inf 表示没有有效回波；过近点多为车体反射，过远点对局部规划没有价值。
      if (!std::isfinite(range) || range < effective_min || range > effective_max) {
        continue;
      }

      const double angle = scan->angle_min + static_cast<double>(index) * scan->angle_increment;
      geometry_msgs::msg::PointStamped point_in_laser;
      point_in_laser.header = scan->header;
      // LaserScan 采用极坐标，按 x=r*cos(theta)、y=r*sin(theta) 转成激光坐标系的点。
      point_in_laser.point.x = range * std::cos(angle);
      point_in_laser.point.y = range * std::sin(angle);
      point_in_laser.point.z = 0.0;

      geometry_msgs::msg::PointStamped point_in_map;
      tf2::doTransform(point_in_laser, point_in_map, transform);
      points.push_back(point_in_map.point);
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = scan->header.stamp;
    cloud.header.frame_id = target_frame_;

    // PointCloud2Modifier 负责正确计算字段偏移、point_step 和 row_step，避免手工拼二进制出错。
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : points) {
      *x = static_cast<float>(point.x);
      *y = static_cast<float>(point.y);
      *z = static_cast<float>(point.z);
      ++x;
      ++y;
      ++z;
    }

    // 空点云也必须发布。它告诉规划器“本帧没有障碍”，从而清除上一帧的旧障碍。
    obstacles_pub_->publish(cloud);
  }

  std::string scan_topic_;
  std::string obstacles_topic_;
  std::string target_frame_;
  double min_range_;
  double max_range_;
  double transform_timeout_;
  int beam_step_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanToObstacles>());
  rclcpp::shutdown();
  return 0;
}
