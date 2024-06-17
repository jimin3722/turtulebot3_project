#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <ctime>
#include <math.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;
using namespace std::chrono_literals;


class main_control : public rclcpp::Node
{
  public:

    main_control() : Node("main_control")
    {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        sub_lidar = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", default_qos, std::bind(&main_control::lidar_callback, this, _1));
        
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", default_qos, std::bind(&main_control::imu_callback, this, _1));

        sub_lidar_angles = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/lidar_angles", 10, std::bind(&main_control::lidar_angles_callback, this, _1));

        sub_velocity = this->create_subscription<std_msgs::msg::Float32>(
        "/velocity", 10, std::bind(&main_control::velocity_callback, this, _1));

        pub_motor = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);

        timer_ = this->create_wall_timer(
        500ms, std::bind(&main_control::control_robot, this));

    }
    
  private:

    float vel = 0.0;
    int angle1 = 90;
    int angle2 = 270;

    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    bool front = false;
    bool right = false;
    bool left = false;

    time_t prev_t = time(0);

    float check_zero(float dis) 
    {
      if(dis < 0.1)
        return 10000.0;
      else
        return dis;
    }

    float abs(float n){
        if(n<0)
            return -n;
        else
            return n;
    }
    
    void lidar_angles_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received_angles: [%d, %d]", msg->data[0], msg->data[1]);

        int a = msg->data[0];
        int b = msg->data[1];
        
        if(a>b){
            angle1 = b;
            angle2 = a;
        }
        else{
            angle2 = b;
            angle1 = a;
        }

    }

    void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received_velocity: %f", msg->data);
        vel = msg->data;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr _msg) 
    {
        // 쿼터니언을 RPY 각도로 변환
        tf2::Quaternion q(_msg->orientation.x, _msg->orientation.y, _msg->orientation.z, _msg->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        roll = roll*180/M_PI;
        pitch = pitch*180/M_PI;
        yaw = yaw*180/M_PI;

        //RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
        //RCLCPP_INFO(this->get_logger(), "lin acc_x = '%f, 'y = '%f, 'z = '%f'\n", _msg->linear_acceleration.x, _msg->linear_acceleration.y, _msg->linear_acceleration.z);
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) //const
    {
        float min_range = 99999.0;
        int min_range_idx = 99999;
        float range; 

        front = false;
        left = false;
        right = false;

        for(int i = 0 ; i<360 ; i++){

            range = _msg->ranges[i];

            if(range < 0.05 || ( i>angle1 && i<angle2 ) )
                continue;
            
            if(range < min_range)
            {
                min_range = range;
                min_range_idx = i;
            }

            if(range < 0.30){
                if( ( 0<=i&&i<=20 ) || ( (360-20)<=i&&i<360 ) )
                    front = true;
                if( 20 < i && i < 180 )
                    left = true;
                if( 180 < i && i < (360-20) )
                    right = true;
            }
            
        }

        if( (time(0) - prev_t) > 1)
        {
            prev_t = time(0);
            int idx = (min_range_idx > 180) ? min_range_idx - 360 : min_range_idx;
            std::cout << idx << " 도 에서 "  << "최소거리 : " << min_range << std::endl;
            std::cout << "현재 roll : " << roll << ", 현재 pitch : " << pitch << ", 현재 yaw : " << yaw << std::endl;
            std::cout <<"---------------------------------------------"<< std::endl;
        }

    }
    
    void control_robot()
    {
        geometry_msgs::msg::Twist twist;

        twist.linear.x = vel;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
        
        if(front || abs(roll) > 60 || abs(pitch) > 60)
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
            if(right && vel > 0.01)
            {
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.5;
            }
            else if(left && vel > 0.01)
            {
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = -0.5;
            }
            
        }
        
        pub_motor->publish(twist);
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_lidar_angles;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_velocity;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_motor;

    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  auto a = std::make_shared<main_control>();

  RCLCPP_INFO(a->get_logger(), "GOOD!!");
  rclcpp::spin(a);
  rclcpp::shutdown();
  return 0;

}

