#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LidarProcessor : public rclcpp::Node
{
public:
    LidarProcessor() : Node("lidar_processor")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&LidarProcessor::lidar_callback, this, std::placeholders::_1));
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Process the point cloud here
        // For example, you can print the number of points
        RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", cloud->size());

        // Add your custom processing logic here
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}