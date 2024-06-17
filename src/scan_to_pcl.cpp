#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

class LaserToPointCloud2 : public rclcpp::Node {
public:
  LaserToPointCloud2() : Node("laser_to_pointcloud2") {

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    pointcloud2_pub_local = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_local", 10);

    pointcloud2_pub_global = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_global", 10);

    accumulated_pointcloud2_pub_global = this->create_publisher<sensor_msgs::msg::PointCloud2>("/accumulated_pcl_global", 10);

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", default_qos, std::bind(&LaserToPointCloud2::laserCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", default_qos, std::bind(&LaserToPointCloud2::odomCallback, this, std::placeholders::_1));
  }

  float x = 0;
  float y = 0;
  float yaw = 0;
  float voxel_size = 0.03;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    sensor_msgs::msg::PointCloud2 pointcloud2_msg;

    // Convert laser scan to PCL point cloud
    for (size_t i = 0; i < msg->ranges.size(); ++i) 
    {
      float angle = msg->angle_min + i * msg->angle_increment;
      float range = msg->ranges[i];
      if (!std::isnan(range) && range < msg->range_max && range > msg->range_min) {
        pcl_cloud.points.emplace_back(range * std::cos(angle) - 0.03 , range * std::sin(angle), 0.0f);
      }
    }

    // Apply voxel grid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(pcl_cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(pcl_cloud_ptr);
    voxel_filter.setLeafSize(voxel_size, voxel_size, 0.1f);
    voxel_filter.filter(*pcl_cloud_filtered);

    // local 좌표 pub
    pcl::toROSMsg(*pcl_cloud_filtered, pointcloud2_msg);
    pointcloud2_msg.header = msg->header;
    pointcloud2_msg.header.frame_id = "map";
    pointcloud2_pub_local->publish(pointcloud2_msg);

    // Create a transformation matrix
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << x, y, 0.0;
    transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

    // Apply the transformation to the point cloud
    pcl::transformPointCloud(*pcl_cloud_filtered, *pcl_cloud_filtered, transform);

    // global 좌표 pub
    pcl::toROSMsg(*pcl_cloud_filtered, pointcloud2_msg);
    pointcloud2_msg.header = msg->header;
    pointcloud2_msg.header.frame_id = "map";
    pointcloud2_pub_global->publish(pointcloud2_msg);

  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    yaw = msg->twist.twist.angular.z;;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_local;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_global;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_pointcloud2_pub_global;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserToPointCloud2>();
  RCLCPP_INFO(node->get_logger(), "Node started");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}