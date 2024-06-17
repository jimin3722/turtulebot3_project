#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <iostream>
#include <vector>

using namespace std;

class Velocity_Publisher : public rclcpp::Node
{
public:
  Velocity_Publisher()
  : Node("multi_integer_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("velocity", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&Velocity_Publisher::publish_velocity, this));
  }

private:
  void publish_velocity()
  {
    std_msgs::msg::Float32 message;
    float vel;

    while(true){
        cout << "Enter velocity: ";
        cin >> vel;
    
        bool range_bool = ( 0 <= vel && vel <= 1 )? true : false; 

        if(range_bool == 0)
            std::cout << "0~1 사이값으로 입력해주세요" << std::endl;
        else
            break;    
    }
    
    message.data = vel;
    RCLCPP_INFO(this->get_logger(), "Publishing: %f", vel);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Velocity_Publisher>());
  rclcpp::shutdown();
  return 0;
}
