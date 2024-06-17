#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <iostream>
#include <vector>

using namespace std;

class MultiIntegerPublisher : public rclcpp::Node
{
public:
  MultiIntegerPublisher()
  : Node("multi_integer_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("multi_ints", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MultiIntegerPublisher::publish_integers, this));
  }

private:
  void publish_integers()
  {
    std_msgs::msg::Int32MultiArray message;
    int int1, int2;
    cout << "Enter two integers: ";
    cin >> int1 >> int2;
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
  rclcpp::spin(std::make_shared<MultiIntegerPublisher>());
  rclcpp::shutdown();
  return 0;
}

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// using namespace std::chrono_literals;

// /* This example creates a subclass of Node and uses std::bind() to register a
// * member function as a callback from the timer. */

// class MinimalPublisher : public rclcpp::Node
// {
//   public:
//     MinimalPublisher()
//     : Node("minimal_publisher"), count_(0)
//     {
//       publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//       timer_ = this->create_wall_timer(
//       500ms, std::bind(&MinimalPublisher::timer_callback, this));ㅋ
//     }

//   private:
//     void timer_callback()
//     {
//       auto message = std_msgs::msg::String();
//       // int id; 

//       // printf("입력 : ");
//       // scanf("%d", &id);
//       // message.data = std::to_string(id);

//       std::string input_string;

//       std::cout << "Enter a string  : ";

//       std::cin >> input_string;

//       //std::getline(std::cin, input_string);

//       message.data = input_string;
      
//       // message.data = "Hello, world! " + std::to_string(count_++);
//       // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//       publisher_->publish(message);
//     }
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//     size_t count_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }
