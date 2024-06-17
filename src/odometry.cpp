#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


#include <ctime>
#include <math.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

double WHEEL_RADIUS = 0.065/2;

class odometry : public rclcpp::Node
{
  public:

    odometry() : Node("odometry")
    {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", default_qos, std::bind(&odometry::imu_callback, this, _1));

        sub_joint_state = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&odometry::joint_state_callback, this, _1));
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("pose_marker", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&odometry::publish_odometry, this));

    }
    
  private:
    
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Quaternion q;

    float wheel_l_rot;
    float wheel_r_rot;

    float global_x = 0, global_y = 0;

    float init_yaw;
    float global_yaw = 0;

    bool imu_flag = false;
    bool joint_state_flag = false;

    time_t prev_t = time(0);

    //odometry 계산
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr _msg)
    {
        float now_wheel_l_rot = _msg->position[0];
        float now_wheel_r_rot = _msg->position[1];
        
        if(joint_state_flag == false){
            wheel_l_rot = now_wheel_l_rot;
            wheel_r_rot = now_wheel_r_rot;
            joint_state_flag = true;
            return;
        }

        float d_wheel_l_rot = now_wheel_l_rot - wheel_l_rot; 
        float d_wheel_r_rot = now_wheel_r_rot - wheel_r_rot;

        wheel_l_rot = now_wheel_l_rot;
        wheel_r_rot = now_wheel_r_rot;

        float d_wheel_l_distance = d_wheel_l_rot*WHEEL_RADIUS;
        float d_wheel_r_distance = d_wheel_r_rot*WHEEL_RADIUS;

        float dx = (d_wheel_l_distance+d_wheel_r_distance)/2*cos(global_yaw);
        float dy = (d_wheel_l_distance+d_wheel_r_distance)/2*sin(global_yaw);

        global_x += dx;
        global_y += dy;

        RCLCPP_INFO(this->get_logger(), "왼쪽 바퀴 변위: %f, 오른쪽 바퀴 변위: %f", d_wheel_l_distance, d_wheel_r_distance);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr _msg) 
    {
        // 쿼터니언을 RPY 각도로 변환
        tf2::Quaternion q(_msg->orientation.x, _msg->orientation.y, _msg->orientation.z, _msg->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        if(imu_flag == false){
            init_yaw = yaw;
            imu_flag = true;
            return;
        }

        global_yaw = yaw - init_yaw;
    }
    
    void publish_odometry() 
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "map";

        odom.pose.pose.position.x = global_x;
        odom.pose.pose.position.y = global_y;
        odom.pose.pose.position.z = 0.0;

        q.setRPY(0, 0, global_yaw);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = global_x;
        odom.twist.twist.linear.y = global_y;
        odom.twist.twist.angular.z = global_yaw;

        // Odometry 데이터 발행
        odom_pub_->publish(odom);
        
        pub_marker();

        // TF 브로드캐스팅
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = odom.pose.pose.position.x;
        tf_msg.transform.translation.y = odom.pose.pose.position.y;
        tf_msg.transform.translation.z = odom.pose.pose.position.z;
        tf_msg.transform.rotation = odom.pose.pose.orientation;
        //tf_broadcaster_->sendTransform(tf_msg);
    }
    
    void pub_marker()
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "pose";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = global_x;
        marker.pose.position.y = global_y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = 0.15;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto a = std::make_shared<odometry>();

  RCLCPP_INFO(a->get_logger(), "GOOD!!");
  rclcpp::spin(a);
  rclcpp::shutdown();
  return 0;
}

