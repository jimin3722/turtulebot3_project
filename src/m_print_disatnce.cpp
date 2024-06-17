#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <ctime>

using std::placeholders::_1;

class laser_read : public rclcpp::Node
{
  public:
    laser_read() 
    : Node("lidar__")
    {
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

      lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", default_qos, std::bind(&laser_read::lidar_callback, this, _1));

    }
    
  private:

    float check_zero(float dis) 
    {
      if(dis < 0.1)
        return 999.0;
      else
        return dis;
    }

    
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) //const
    {
      float front, back;

      front = check_zero(_msg->ranges[0]);
      back = check_zero(_msg->ranges[180]);

      std::cout << "front :" << front << std::endl;
      std::cout << "back :" << back << std::endl;
      std::cout << "--------------" << std::endl;

    }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_;

};


int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  auto a = std::make_shared<laser_read>();

  RCLCPP_INFO(a->get_logger(), "GOOD!!");
  rclcpp::spin(a);
  rclcpp::shutdown();
  return 0;

}

