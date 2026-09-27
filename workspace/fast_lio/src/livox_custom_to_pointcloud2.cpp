// 原：无此文件
// 改：新增 Livox CustomMsg -> PointCloud2 旁路转换节点，仅用于 RViz 显示原始雷达点云。
// 作用：订阅 /livox/lidar(CustomMsg)，发布 /livox/lidar_points(PointCloud2)，不改变 FAST-LIO 建图输入。

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LivoxCustomToPointCloud2 : public rclcpp::Node
{
public:
    LivoxCustomToPointCloud2() : Node("livox_custom_to_pointcloud2")
    {
        // 原：无转换节点参数
        // 改：新增输入/输出话题和输出坐标系参数，默认输出到 FAST-LIO 的 body 帧，便于 camera_init 固定帧下显示。
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/livox/lidar");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/livox/lidar_points");
        output_frame_ = this->declare_parameter<std::string>("output_frame", "body");
        lidar_to_body_translation_ = this->declare_parameter<std::vector<double>>(
            "lidar_to_body_translation", {-0.011, -0.02329, 0.04412});
        if (lidar_to_body_translation_.size() != 3)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "lidar_to_body_translation must have 3 values, fallback to [0, 0, 0]");
            lidar_to_body_translation_ = {0.0, 0.0, 0.0};
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, rclcpp::SensorDataQoS());
        subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LivoxCustomToPointCloud2::cloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Converting Livox CustomMsg %s -> PointCloud2 %s in frame %s for RViz display",
            input_topic_.c_str(), output_topic_.c_str(), output_frame_.c_str());
    }

private:
    void cloudCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
    {
        // 原：RViz 不能直接显示 /livox/lidar 的 CustomMsg
        // 改：将 CustomMsg 中的 x/y/z/reflectivity 转为 PointCloud2 的 x/y/z/intensity。
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = msg->header;
        if (!output_frame_.empty())
        {
            cloud.header.frame_id = output_frame_;
        }
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

        for (const auto & point : msg->points)
        {
            // 原：原始点云位于 livox_frame
            // 改：应用 mid360.yaml 中 LiDAR->IMU 平移外参后发布到 body 帧，仅用于显示对齐。
            *iter_x = point.x + static_cast<float>(lidar_to_body_translation_[0]);
            *iter_y = point.y + static_cast<float>(lidar_to_body_translation_[1]);
            *iter_z = point.z + static_cast<float>(lidar_to_body_translation_[2]);
            *iter_intensity = static_cast<float>(point.reflectivity);
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_intensity;
        }

        publisher_->publish(cloud);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string output_frame_;
    std::vector<double> lidar_to_body_translation_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxCustomToPointCloud2>());
    rclcpp::shutdown();
    return 0;
}
