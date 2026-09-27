// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pcd2pgm/pcd2pgm.hpp"

#include <fstream>
#include <limits>

#include "pcl/common/transforms.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"

namespace pcd2pgm
{
Pcd2PgmNode::Pcd2PgmNode(const rclcpp::NodeOptions & options) : Node("pcd2pgm", options)
{
  declareParameters();
  getParameters();

  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);

  pcd_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name_, map_qos);
  pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd_cloud", 10);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *pcd_cloud_) == -1) {
    RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Initial point cloud size: %lu", pcd_cloud_->points.size());

  applyTransform();

  passThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
  radiusOutlierFilter(cloud_after_pass_through_, thre_radius_, thres_point_count_);
  setMapTopicMsg(cloud_after_radius_, map_topic_msg_);
  if (save_to_file_) {
    saveMapFiles();
  }

  timer_ =
    create_wall_timer(std::chrono::seconds(1), std::bind(&Pcd2PgmNode::publishCallback, this));
}

void Pcd2PgmNode::publishCallback()
{
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud_after_radius_, output);
  output.header.frame_id = "map";
  pcd_publisher_->publish(output);
  map_publisher_->publish(map_topic_msg_);
}

void Pcd2PgmNode::declareParameters()
{
  declare_parameter("pcd_file", "");
  declare_parameter("thre_z_min", 0.5);
  declare_parameter("thre_z_max", 2.0);
  declare_parameter("flag_pass_through", false);
  declare_parameter("thre_radius", 0.5);
  declare_parameter("map_resolution", 0.05);
  declare_parameter("thres_point_count", 10);
  declare_parameter("min_points_per_cell", 1);
  declare_parameter("map_topic_name", "map");
  declare_parameter("save_to_file", false);
  declare_parameter("output_map_yaml", "");
  declare_parameter("output_map_image", "");
  declare_parameter(
    "odom_to_lidar_odom", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});  // 新增的参数
}

void Pcd2PgmNode::getParameters()
{
  get_parameter("pcd_file", pcd_file_);
  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("min_points_per_cell", min_points_per_cell_);
  get_parameter("map_topic_name", map_topic_name_);
  get_parameter("save_to_file", save_to_file_);
  get_parameter("output_map_yaml", output_map_yaml_);
  get_parameter("output_map_image", output_map_image_);
  get_parameter("odom_to_lidar_odom", odom_to_lidar_odom_);  // 获取新的参数
}

void Pcd2PgmNode::passThroughFilter(double thre_low, double thre_high, bool flag_in)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud_);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*filtered_cloud);

  cloud_after_pass_through_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After PassThrough filtering: %lu points",
    cloud_after_pass_through_->points.size());
}

void Pcd2PgmNode::radiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, double radius, int thre_count)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
  radius_outlier.setInputCloud(input_cloud);
  radius_outlier.setRadiusSearch(radius);
  radius_outlier.setMinNeighborsInRadius(thre_count);
  radius_outlier.filter(*filtered_cloud);

  cloud_after_radius_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After RadiusOutlier filtering: %lu points", cloud_after_radius_->points.size());
}

void Pcd2PgmNode::setMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = now();
  msg.info.resolution = map_resolution_;

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  if (cloud->points.empty()) {
    RCLCPP_WARN(get_logger(), "Point cloud is empty!");
    return;
  }

  for (const auto & point : cloud->points) {
    x_min = std::min(x_min, static_cast<double>(point.x));
    x_max = std::max(x_max, static_cast<double>(point.x));
    y_min = std::min(y_min, static_cast<double>(point.y));
    y_max = std::max(y_max, static_cast<double>(point.y));
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = std::ceil((x_max - x_min) / map_resolution_);
  msg.info.height = std::ceil((y_max - y_min) / map_resolution_);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  std::vector<uint16_t> cell_counts(msg.info.width * msg.info.height, 0);

  for (const auto & point : cloud->points) {
    int i = std::floor((point.x - x_min) / map_resolution_);
    int j = std::floor((point.y - y_min) / map_resolution_);

    if (i >= 0 && i < msg.info.width && j >= 0 && j < msg.info.height) {
      const auto index = static_cast<size_t>(i) + static_cast<size_t>(j) * msg.info.width;
      if (cell_counts[index] < std::numeric_limits<uint16_t>::max()) {
        ++cell_counts[index];
      }
    }
  }

  size_t occupied_cells = 0;
  for (size_t index = 0; index < cell_counts.size(); ++index) {
    if (cell_counts[index] >= min_points_per_cell_) {
      msg.data[index] = 100;
      ++occupied_cells;
    }
  }

  RCLCPP_INFO(
    get_logger(), "Map data size: %lu occupied_cells: %lu min_points_per_cell: %d",
    msg.data.size(), occupied_cells, min_points_per_cell_);
}

void Pcd2PgmNode::applyTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  transform.translation() << odom_to_lidar_odom_[0], odom_to_lidar_odom_[1], odom_to_lidar_odom_[2];
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[3], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[4], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[5], Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*pcd_cloud_, *pcd_cloud_, transform.inverse());
}

bool Pcd2PgmNode::saveMapFiles()
{
  if (output_map_yaml_.empty() || output_map_image_.empty()) {
    RCLCPP_ERROR(
      get_logger(), "save_to_file is true but output_map_yaml or output_map_image is empty");
    return false;
  }
  if (map_topic_msg_.data.empty() || map_topic_msg_.info.width == 0 || map_topic_msg_.info.height == 0) {
    RCLCPP_ERROR(get_logger(), "Cannot save empty occupancy grid");
    return false;
  }

  std::ofstream pgm(output_map_image_, std::ios::binary);
  if (!pgm.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open PGM output: %s", output_map_image_.c_str());
    return false;
  }

  const auto width = map_topic_msg_.info.width;
  const auto height = map_topic_msg_.info.height;
  pgm << "P5\n" << width << " " << height << "\n255\n";
  for (int y = static_cast<int>(height) - 1; y >= 0; --y) {
    for (uint32_t x = 0; x < width; ++x) {
      const int8_t value = map_topic_msg_.data[static_cast<size_t>(x) + static_cast<size_t>(y) * width];
      unsigned char pixel = 205;
      if (value >= 65) {
        pixel = 0;
      } else if (value >= 0) {
        pixel = 254;
      }
      pgm.write(reinterpret_cast<const char *>(&pixel), 1);
    }
  }
  pgm.close();

  std::string image_name = output_map_image_;
  const auto slash = image_name.find_last_of("/\\");
  if (slash != std::string::npos) {
    image_name = image_name.substr(slash + 1);
  }

  std::ofstream yaml(output_map_yaml_);
  if (!yaml.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open YAML output: %s", output_map_yaml_.c_str());
    return false;
  }
  yaml << "image: " << image_name << "\n";
  yaml << "mode: trinary\n";
  yaml << "resolution: " << map_topic_msg_.info.resolution << "\n";
  yaml << "origin: ["
       << map_topic_msg_.info.origin.position.x << ", "
       << map_topic_msg_.info.origin.position.y << ", 0.0]\n";
  yaml << "negate: 0\n";
  yaml << "occupied_thresh: 0.65\n";
  yaml << "free_thresh: 0.25\n";
  yaml.close();

  RCLCPP_INFO(
    get_logger(), "Saved Nav2 map files: %s and %s",
    output_map_yaml_.c_str(), output_map_image_.c_str());
  return true;
}

}  // namespace pcd2pgm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::Pcd2PgmNode)
