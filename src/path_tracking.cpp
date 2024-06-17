#include <iostream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <ctime>

using namespace std::chrono_literals;

int LD = 6;

struct Vec2i
{
    float x, y;
};

class Path_tracker : public rclcpp::Node {
public:
  Path_tracker() : Node("path_tracker") {

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    motor_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);

    ld_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
    "/ld_point_marker", 10,
    std::bind(&Path_tracker::ld_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry", default_qos, std::bind(&Path_tracker::odomCallback, this, std::placeholders::_1));

    path_tracker_ = this->create_wall_timer(
    10ms, std::bind(&Path_tracker::control_robot, this));    

  }

    time_t prev_t = time(0);
    time_t check_t = time(0);
    time_t back_t = 0;

    float prev_global_x = 0;
    float prev_global_y = 0;

    float global_x = 99;
    float global_y = 99;
    float global_yaw = 0;

    float LD_x = 0;
    float LD_y = 0;
    float LD_yaw = 0;  


  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
  {
    global_x = msg->twist.twist.linear.x;
    global_y = msg->twist.twist.linear.y;
    global_yaw = msg->twist.twist.angular.z;
    
    prev_t = time(0); 

  }  

  void control_robot()
    {
      // 두 점 사이의 각도 계산
      double delta_x = LD_x - global_x;
      double delta_y = LD_y - global_y;

      LD_yaw = std::atan2(delta_y, delta_x);    

      float gy = (global_yaw < 0)? global_yaw + 2*M_PI : global_yaw;
      float ly = (LD_yaw < 0)? LD_yaw + 2*M_PI : LD_yaw;

      //gy = global_yaw + 2*M_PI;
      ly = LD_yaw;

      std::cout << ly << " : LD_yaw" << std::endl;
      std::cout << gy << " : global_yaw" << std::endl;      

      double d_yaw = ly - gy;

      d_yaw = (d_yaw > M_PI)? d_yaw - 2*M_PI : d_yaw;
      d_yaw = (d_yaw < -M_PI)? d_yaw + 2*M_PI : d_yaw;

      std::cout << d_yaw << " : d_yaw" << std::endl; 

      
      float lin_vel = 0.1;
      float ang_vel = 0.0;
      
      if(abs(d_yaw) < 7*M_PI/180)
      { 
        lin_vel = 0.2;
        ang_vel = 0.0;
      }
      else
      {
          if(d_yaw > 0)
          {
            lin_vel = 0.0;
            ang_vel = 0.5;
          }
          else 
          {
            lin_vel = 0.0;
            ang_vel = -0.5;
          }
      }
    
    // 일정 시간 이상 odom값 안들어오면 0,0 
    if(time(0) - prev_t > 3)
    {
        lin_vel = 0;
        ang_vel = 0;
    }
    
    if((time(0) - check_t) > 6)
    {
      if(thinking())
        back_t = time(0);

      prev_global_x = global_x;
      prev_global_y = global_y;
      check_t = time(0);

    }

    if(time(0) - back_t < 2)
    {
        lin_vel = -lin_vel;
        ang_vel = 0;
    }

      geometry_msgs::msg::Twist twist;

      twist.linear.x = lin_vel;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = ang_vel;
      
      motor_pub_->publish(twist);
    }

    void ld_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
    {
        LD_x = msg->pose.position.x;
        LD_y = msg->pose.position.y;

    }

    bool thinking()
    {
      float distance  = sqrt(pow((global_x-prev_global_x),2)+pow((global_y-prev_global_y),2));
      
      if(distance < 0.1)
        return true;
      else
        return false;
    }

private:

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_ ;

  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr ld_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr path_tracker_;

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Path_tracker>();
  RCLCPP_INFO(node->get_logger(), "GOOD!!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

