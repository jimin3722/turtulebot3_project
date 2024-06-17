#include <iostream>
#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>


#include "../include/turtlebot3_project/AStar.hpp"

using namespace std::chrono_literals;

int LD = 4;//5

struct Vec2i
{
    float x, y;
};

class Path_Generator : public rclcpp::Node {
public:
  Path_Generator() 
  : Node("path_generator"), 
    generator() // 멤버 초기화 리스트에서 generator 초기화
  {

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path_topic", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ld_point_marker", 10);

    point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pcl_global", 10,
    std::bind(&Path_Generator::pointcloud_callback, this, std::placeholders::_1));

    // accumulated_point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    // "/accumulated_pcl_global", 10,
    // std::bind(&Path_Generator::accumulated_pointcloud_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry", default_qos, std::bind(&Path_Generator::odomCallback, this, std::placeholders::_1));

    path_planner_ = this->create_wall_timer(
    100ms, std::bind(&Path_Generator::generating_path, this));

    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setWorldSize({0, 9}, {-0.8, 0.8});
    generator.setBackMovement(false);
    generator.setGridSize(0.05);
    generator.setCollisionDis(0.18);//17
    generator.setCollisionDis_main(0.18);
    generator.setCollisionDis_sub(0.12);//
  }

  float global_x = 0;
  float global_y = 0;
  float global_yaw = 0;

  float LD_x = 0;
  float LD_y = 0;

  float LD_yaw = 0;  

  std::vector<std::vector<float>> point_vector;
  std::vector<std::vector<float>> accumulated_point_vector;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
  {
    global_x = msg->twist.twist.linear.x;
    global_y = msg->twist.twist.linear.y;
    global_yaw = msg->twist.twist.angular.z;
  }  
  
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    // pcl::PointCloud 객체를 생성합니다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // sensor_msgs::PointCloud2를 pcl::PointCloud<pcl::PointXYZ>로 변환합니다.
    pcl::fromROSMsg(*msg, *cloud);

    // 변환된 포인트 클라우드를 벡터로 변환합니다.
    point_vector.clear();

    for (const auto& point : cloud->points) 
    {
      std::vector<float> temp_point;
      temp_point.push_back(point.x); // x 좌표 추가
      temp_point.push_back(point.y); // y 좌표 추가
      point_vector.push_back(temp_point);
    }

    std::cout << point_vector.size() << "point_vector.size()" << std::endl; 

  }

  void accumulated_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    // pcl::PointCloud 객체를 생성합니다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // sensor_msgs::PointCloud2를 pcl::PointCloud<pcl::PointXYZ>로 변환합니다.
    pcl::fromROSMsg(*msg, *cloud);

    accumulated_point_vector.clear();

    for (const auto& point : cloud->points) 
    {
      std::vector<float> temp_point;
      temp_point.push_back(point.x); // x 좌표 추가
      temp_point.push_back(point.y); // y 좌표 추가
      accumulated_point_vector.push_back(temp_point);
    }

  }
      
  void generating_path()
  {
    generator.clearCollisions();
    generator.addCollision(point_vector);
    //generator.addCollision(accumulated_point_vector);

    auto start = std::chrono::high_resolution_clock::now();
    
    std::cout << "Generate path ... \n";
    auto path = generator.findPath({global_x, global_y}, {3 + global_x, 0});

    // 종료 시간 기록
    auto end = std::chrono::high_resolution_clock::now();
    // 시작과 종료 시간의 차이를 계산 (밀리초 단위)
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "dt : " << duration << " ms" << std::endl;

    size_t size = path.size();

    // plan B >> A*가 경로를 생성 못할 시 벽따라가는 경로 생성
    if (size < LD) {

        std::cerr << "벡터의 크기가"<< LD <<"보다 작습니다." << std::endl;

        float step_size = 0.01;
        float LD = 0.25;
        float collision_dis = 0.15;
        int max_steps = 1000;
        
        auto path = followWall(step_size, max_steps, LD, collision_dis);
        
        size_t size = path.size();

        auto LD_point = path[size];
        LD_x = LD_point.x;
        LD_y = LD_point.y;

        pub_marker();

        auto path_msg = std::make_shared<nav_msgs::msg::Path>();
        path_msg->header.stamp = this->now();
        path_msg->header.frame_id = "map"; 

        for(auto& coordinate : path) {
          
          geometry_msgs::msg::PoseStamped pose;
          pose.header.stamp = this->now();
          pose.header.frame_id = "map";
          pose.pose.position.x = coordinate.x;
          pose.pose.position.y = coordinate.y;
          pose.pose.position.z = 0.0;
          pose.pose.orientation.w = 1.0;

          path_msg->poses.push_back(pose);

        }

    path_pub_->publish(*path_msg);

        return;
    }

    size_t index = size - LD;

    // 뒤에서 10번째 요소 가져오기
    auto LD_point = path[index];
    LD_x = LD_point.x;
    LD_y = LD_point.y;

    pub_marker();

    auto path_msg = std::make_shared<nav_msgs::msg::Path>();
    path_msg->header.stamp = this->now();
    path_msg->header.frame_id = "map"; // 적절한 프레임 ID를 설정하세요

    for(auto& coordinate : path) {
      
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      pose.pose.position.x = coordinate.x;
      pose.pose.position.y = coordinate.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path_msg->poses.push_back(pose);

    }

    path_pub_->publish(*path_msg);
  }


  // 충돌을 확인하는 함수
  bool checkCollision(float x, float y, float collision_dis) {
      for (const auto& point : point_vector) {
          float distance = std::sqrt(std::pow(point[0] - x, 2) + std::pow(point[1] - y, 2));
          if (distance < collision_dis) {
              return true;
          }
      }
      return false;
  }

  // 벽을 따라가는 경로를 생성하는 함수
  std::vector<Vec2i> followWall(float step_size, int max_steps, float LD, float collision_dis) 
  {
      std::vector<Vec2i> path;

      float x = global_x;
      float y = global_y;
      float direction_x = 1.0; // 초기 방향 오른쪽
      float direction_y = 0.0;
      float traveled_distance = 0.0;

      for (int i = 0; i < max_steps; ++i) {
          path.push_back({x, y});
          traveled_distance += step_size;

          if (traveled_distance >= LD) {
              break;
          }

          // 오른손 법칙에 따라 이동 방향 결정
          float right_x = direction_y;
          float right_y = -direction_x;
          float new_x = x + right_x * step_size;
          float new_y = y + right_y * step_size;

          if (!checkCollision(new_x, new_y, collision_dis)) {
              // 오른쪽으로 회전
              direction_x = right_x;
              direction_y = right_y;
              x = new_x;
              y = new_y;
          } else {
              // 직진
              x += direction_x * step_size;
              y += direction_y * step_size;
          }
      }

      return path;
  }

  void pub_marker()
  {
    // 마커 메시지 생성
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "ld_point";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = LD_x;
    marker.pose.position.y = LD_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;  // 크기 설정
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;  // 불투명도 설정
    marker.color.r = 1.0;  // 빨간색
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // 마커 퍼블리시
    marker_pub_->publish(marker);
  }

private:

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_point_sub_;

  rclcpp::TimerBase::SharedPtr path_planner_;

  AStar::Generator generator; 
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Path_Generator>();
  RCLCPP_INFO(node->get_logger(), "GOOD!!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

