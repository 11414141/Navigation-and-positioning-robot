#include <cstdint>
#include <functional>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LivoxCustomToPointCloud2 : public rclcpp::Node
{
public:
  LivoxCustomToPointCloud2() : Node("livox_custom_to_pointcloud2")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar_points");
    nav_output_topic_ = declare_parameter<std::string>("nav_output_topic", "/livox/lidar_points_nav");
    glim_output_topic_ = declare_parameter<std::string>("glim_output_topic", "/livox/lidar_points_glim");
    output_frame_ = declare_parameter<std::string>("output_frame", "livox_frame");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    angle_min_ = declare_parameter<double>("angle_min", -3.1416);
    angle_max_ = declare_parameter<double>("angle_max", 3.1416);
    angle_increment_ = declare_parameter<double>("angle_increment", 0.0087);
    range_min_ = declare_parameter<double>("range_min", 0.4);
    range_max_ = declare_parameter<double>("range_max", 50.0);
    min_height_ = declare_parameter<double>("min_height", 0.0);
    max_height_ = declare_parameter<double>("max_height", 0.15);
    use_inf_ = declare_parameter<bool>("use_inf", false);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    nav_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      nav_output_topic_, rclcpp::SensorDataQoS());
    glim_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      glim_output_topic_, rclcpp::SensorDataQoS());
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LivoxCustomToPointCloud2::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Converting Livox CustomMsg %s -> nav PointCloud2 %s/%s, GLIM PointCloud2 %s, and LaserScan %s in frame %s",
      input_topic_.c_str(), output_topic_.c_str(), nav_output_topic_.c_str(),
      glim_output_topic_.c_str(), scan_topic_.c_str(), output_frame_.c_str());
  }

private:
  void cloudCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
  {
    const auto stamp = now();

    const auto frame_id = output_frame_.empty() ? msg->header.frame_id : output_frame_;

    auto nav_cloud = makeBasicCloud(msg, stamp, frame_id);
    publisher_->publish(nav_cloud);
    nav_publisher_->publish(nav_cloud);

    auto glim_cloud = makeGlimCloud(msg, frame_id);
    glim_publisher_->publish(glim_cloud);

    publishScan(msg, stamp, frame_id);
  }

  sensor_msgs::msg::PointCloud2 makeBasicCloud(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg,
    const rclcpp::Time & stamp,
    const std::string & frame_id)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(msg->points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(msg->points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

    for (const auto & point : msg->points) {
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      *iter_intensity = static_cast<float>(point.reflectivity);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }

    return cloud;
  }

  sensor_msgs::msg::PointCloud2 makeGlimCloud(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg,
    const std::string & frame_id)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(msg->points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      8,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32,
      "timestamp", 1, sensor_msgs::msg::PointField::FLOAT32,
      "offset_time", 1, sensor_msgs::msg::PointField::UINT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16);
    modifier.resize(msg->points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> iter_time(cloud, "time");
    sensor_msgs::PointCloud2Iterator<float> iter_timestamp(cloud, "timestamp");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_offset_time(cloud, "offset_time");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(cloud, "ring");

    for (const auto & point : msg->points) {
      const float offset_seconds = static_cast<float>(point.offset_time) * 1.0e-9F;
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      *iter_intensity = static_cast<float>(point.reflectivity);
      *iter_time = offset_seconds;
      *iter_timestamp = offset_seconds;
      *iter_offset_time = point.offset_time;
      *iter_ring = static_cast<uint16_t>(point.line);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
      ++iter_time;
      ++iter_timestamp;
      ++iter_offset_time;
      ++iter_ring;
    }

    return cloud;
  }

  void publishScan(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg,
    const rclcpp::Time & stamp,
    const std::string & frame_id)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = frame_id;
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_max_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = 0.0F;
    scan.scan_time = 0.0F;
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);

    const int beam_count = static_cast<int>(
      std::ceil((angle_max_ - angle_min_) / angle_increment_));
    if (beam_count <= 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Invalid LaserScan angular parameters; skip scan publish");
      return;
    }

    const float no_return = use_inf_ ?
      std::numeric_limits<float>::infinity() :
      static_cast<float>(range_max_ + 1.0);
    scan.ranges.assign(static_cast<size_t>(beam_count), no_return);

    for (const auto & point : msg->points) {
      if (point.z < min_height_ || point.z > max_height_) {
        continue;
      }

      const double range = std::hypot(point.x, point.y);
      if (range < range_min_ || range > range_max_) {
        continue;
      }

      const double angle = std::atan2(point.y, point.x);
      if (angle < angle_min_ || angle > angle_max_) {
        continue;
      }

      const int index = static_cast<int>((angle - angle_min_) / angle_increment_);
      if (index < 0 || index >= beam_count) {
        continue;
      }

      auto & current = scan.ranges[static_cast<size_t>(index)];
      if (!std::isfinite(current) || range < current) {
        current = static_cast<float>(range);
      }
    }

    scan_publisher_->publish(scan);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string nav_output_topic_;
  std::string glim_output_topic_;
  std::string output_frame_;
  std::string scan_topic_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double range_min_;
  double range_max_;
  double min_height_;
  double max_height_;
  bool use_inf_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nav_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr glim_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxCustomToPointCloud2>());
  rclcpp::shutdown();
  return 0;
}
