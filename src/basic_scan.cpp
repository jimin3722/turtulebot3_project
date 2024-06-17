#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <ctime>

using std::placeholders::_1;

float vel = 0.0;
float ori = 0.0;

time_t prev_t = time(0);

class laser_read : public rclcpp::Node
{
  public:
    laser_read() : Node("lidar__")
    {
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

      lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", default_qos, std::bind(&laser_read::lidar_callback, this, _1));

      subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "multi_ints", 10, std::bind(&laser_read::multi_ints_callback, this, _1));

      motor_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);

    }
    
  private:

    float check_zero(float dis) 
    {
      if(dis < 0.1)
        return 10000.0;
      else
        return dis;
    }
    
    void multi_ints_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: [%d, %d]", msg->data[0], msg->data[1]);

        vel = msg->data[0];
        ori = msg->data[1];
    }

    
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) //const
    {
      float front, right, left;

      front = check_zero(_msg->ranges[0]);
      left = check_zero(_msg->ranges[45]);
      right = check_zero(_msg->ranges[315]);

      std::cout << "front :" << front << std::endl;
      std::cout << "right :" << right << std::endl;
      std::cout << "left :" << left << std::endl;
      std::cout << "--------------" << std::endl;

      geometry_msgs::msg::Twist twist;

      twist.linear.x = vel;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = ori;


      if(front < 0.3)
      { 
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
      }

      else
      {
        if(right < 0.3)
        {
          twist.linear.x = 0.0;
          twist.linear.y = 0.0;
          twist.linear.z = 0.0;
          twist.angular.x = 0.0;
          twist.angular.y = 0.0;
          twist.angular.z = 1;
        }
        else if(left < 0.3)
        {
          twist.linear.x = 0.0;
          twist.linear.y = 0.0;
          twist.linear.z = 0.0;
          twist.angular.x = 0.0;
          twist.angular.y = 0.0;
          twist.angular.z = -1;
        }
        
      }
    
    motor_->publish(twist);

    }


  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_;

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


// class laser_read : public rclcpp::Node
// {
//   public:
//     laser_read() : Node("lidar__")
//     {
//       auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

//       lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//         "/scan", default_qos, std::bind(&laser_read::lidar_callback, this, _1));

//       subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
//       "multi_ints", 10, std::bind(&laser_read::multi_ints_callback, this, _1));
//     }
    
//   private:
    
//     void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) const
//     {


//     if( (time(0) - prev_t) < 0.5)
//         return;
//     else
//         prev_t = time(0); 


//     if(_msg->ranges[a] == 0 || _msg->ranges[b] == 0)
//     {
//         printf("장애물 없음\n");
//         RCLCPP_INFO(this->get_logger(), "['%d'] = '%f' ['%d'] = '%f'", a, 9999999.0, b, 9999999.0);

//         return;
//     }

//     if(_msg->ranges[a] < 0.4 || _msg->ranges[b] < 0.4)
//     {
//       printf("장애물 인지\n");
//       RCLCPP_INFO(this->get_logger(), "['%d'] = '%f' ['%d'] = '%f'", a, _msg->ranges[a], b, _msg->ranges[b]);
//     }
    
//     else
//     {
//       printf("장애물 없음\n");
//       RCLCPP_INFO(this->get_logger(), "['%d'] = '%f' ['%d'] = '%f'", a, _msg->ranges[a], b, _msg->ranges[b]);
//     }

//     }

//     void multi_ints_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
//     {
//         RCLCPP_INFO(this->get_logger(), "Received: [%d, %d]", msg->data[0], msg->data[1]);

//         a = msg->data[0];
//         b = msg->data[1];
//     }

//   rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_;

// };



// int main(int argc, char * argv[])
// {

//   rclcpp::init(argc, argv);
//   auto a = std::make_shared<laser_read>();

//   //RCLCPP_INFO(a->get_logger(), "GOOD!!");
//   rclcpp::spin(a);
//   rclcpp::shutdown();
//   return 0;
// }


// class MultiIntegerSubscriber : public rclcpp::Node
// {
// public:
//   MultiIntegerSubscriber()
//   : Node("multi_integer_subscriber")
//   {
//     subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
//       "multi_ints", 10, std::bind(&MultiIntegerSubscriber::multi_ints_callback, this, _1));
//   }

// private:
//   void multi_ints_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received: [%d, %d]", msg->data[0], msg->data[1]);

//     a = msg->data[0];
//     b = msg->data[1];
//   }
//   rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
// };


// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"

// #define RAD2DEG(x) ((x)*180./M_PI)

// rclcpp::Node::SharedPtr g_node = nullptr;

// void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
// {
//     int count = scan->scan_time / scan->time_increment;

//     printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
//     printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
//     for(int i = 0; i < count; i++) 
//     {
//         float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
// 	    if(degree > -5 && degree< 5)
//         {
//             printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);
//         }
//     }
// }

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     g_node = rclcpp::Node::make_shared("ydlidar_client");
//     auto sub = g_node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1000, scanCallback);
//     rclcpp::spin(g_node);
//     rclcpp::shutdown();

//     return 0;
// }