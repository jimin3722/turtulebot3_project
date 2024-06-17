#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LidarToPclNode : public rclcpp::Node
{
public:
  LidarToPclNode() : Node("lidar_to_pcl_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster/points", 10, std::bind(&LidarToPclNode::lidarCallback, this, std::placeholders::_1));
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // 여기서부터는 PCL을 사용하여 데이터를 처리하는 코드를 작성할 수 있습니다.

    // 예시: 포인트 클라우드의 크기 출력
    RCLCPP_INFO(this->get_logger(), "Received point cloud with %lu points", cloud->size());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarToPclNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}