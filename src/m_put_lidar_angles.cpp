#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <iostream>
#include <vector>

using namespace std;

class Lidar_angles_Publisher : public rclcpp::Node
{
public:
  Lidar_angles_Publisher()
  : Node("multi_integer_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("lidar_angles", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&Lidar_angles_Publisher::publish_angles, this));
  }

private:

  bool check_range(int n)
  {
    if(n<0)
        n*=-1;
    if( 0<=n && n<=180 )
        return true;
    else
        return false;
  }

  void publish_angles()
  {
    std_msgs::msg::Int32MultiArray message;
    int int1, int2;
    bool range_bool; 
        
    while(true){
        cout << "Enter two lidar_angles: ";
        cin >> int1 >> int2;
        
        bool range_bool = ( check_range(int1) && check_range(int2) )? true : false; 

        if(range_bool == 0)
            std::cout << "-180~180 사이값으로 입력해주세요" << std::endl;
        else
            break;    
    }
    
    if(int1 < 0)
        int1 += 360;
    if(int2 < 0)
        int2 += 360;

    message.data = {int1, int2};
    RCLCPP_INFO(this->get_logger(), "Publishing: [%d, %d]", int1, int2);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lidar_angles_Publisher>());
  rclcpp::shutdown();
  return 0;
}



