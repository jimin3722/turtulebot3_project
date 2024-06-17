
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"


using std::placeholders::_1;

class imu_read: public rclcpp::Node
{
  public:
    imu_read() : Node("imu_")
    {
      //auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
      auto default_qos = rclcpp::SensorDataQoS();

      imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", default_qos,
        std::bind(&imu_read::lidar_callback, this, _1));

    }

  private:
    void lidar_callback(const sensor_msgs::msg::Imu::SharedPtr _msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "[0th] = '%f' [100th] = '%f'", _msg->ranges[0], _msg->ranges[100]);
      RCLCPP_INFO(this->get_logger(), "orientation_x = '%f' y:'%f' z:'%f'", _msg->orientation.x, _msg->orientation.y, _msg->orientation.z);
      RCLCPP_INFO(this->get_logger(), "lin acc_x = '%f, 'y = '%f, 'z = '%f'\n", _msg->linear_acceleration.x, _msg->linear_acceleration.y, _msg->linear_acceleration.z);
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;

};

int main(int argc, char * argv[])
{
  //int aaa = 1;
  rclcpp::init(argc, argv);
  auto a = std::make_shared<imu_read>();
  RCLCPP_INFO(a->get_logger(), "GOOD!!");
  rclcpp::spin(a);
  rclcpp::shutdown();
  return 0;

}


