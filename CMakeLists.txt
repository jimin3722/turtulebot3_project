cmake_minimum_required(VERSION 3.5)
project(turtlebot3_project)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# <PackageName>_CONFIG 변수: 
# find_package가 패키지를 찾은 후, 해당 패키지의 설정 파일 경로는 <PackageName>_CONFIG 변수에 저장됩니다. 

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(example_interfaces REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_library(${PROJECT_NAME}_lib
  "src/AStar.cpp"
)


## 과제
add_executable(basic_publisher src/basic_publisher.cpp)
ament_target_dependencies(basic_publisher rclcpp std_msgs)#의존성 설정
add_executable(basic_subscriber src/basic_subscriber.cpp)
ament_target_dependencies(basic_subscriber rclcpp std_msgs)#의존성 설정
add_executable(basic_pcl src/basic_pcl.cpp)
ament_target_dependencies(basic_pcl rclcpp pcl_conversions)#의존성 설정
add_executable(basic_scan src/basic_scan.cpp)
ament_target_dependencies(basic_scan rclcpp std_msgs sensor_msgs geometry_msgs)#의존성 설정
add_executable(basic_imu src/basic_imu.cpp)
ament_target_dependencies(basic_imu rclcpp std_msgs sensor_msgs)
add_executable(basic_motor src/motor_control.cpp)
ament_target_dependencies(basic_motor rclcpp std_msgs sensor_msgs geometry_msgs)

## 중간고사
add_executable(m_print_disatnce src/m_print_disatnce.cpp)
ament_target_dependencies(m_print_disatnce rclcpp std_msgs sensor_msgs geometry_msgs)
add_executable(m_main_control src/m_main_control.cpp)
ament_target_dependencies(m_main_control rclcpp std_msgs sensor_msgs geometry_msgs tf2)
add_executable(m_put_velocity src/m_put_velocity.cpp)
ament_target_dependencies(m_put_velocity rclcpp std_msgs)#의존성 설정
add_executable(m_put_lidar_angles src/m_put_lidar_angles.cpp)
ament_target_dependencies(m_put_lidar_angles rclcpp std_msgs)#의존성 설정

## 기말 텀 프로젝트
add_executable(localization src/odometry.cpp)
ament_target_dependencies(localization rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros visualization_msgs)

add_executable(scan_to_pcl src/scan_to_pcl.cpp)
ament_target_dependencies(scan_to_pcl rclcpp sensor_msgs pcl_conversions nav_msgs)#의존성 설정

add_executable(path_planner src/path_planning.cpp)
target_link_libraries(path_planner ${PROJECT_NAME}_lib)
ament_target_dependencies(path_planner rclcpp sensor_msgs pcl_conversions nav_msgs visualization_msgs)#의존성 설정

add_executable(path_tracker src/path_tracking.cpp)
ament_target_dependencies(path_tracker rclcpp sensor_msgs nav_msgs visualization_msgs)#의존성 설정

target_link_libraries(basic_pcl#
  ${PCL_LIBRARIES}
#   ${rclcpp_lifecycle_LIBRARIES}
#   ${std_msgs_LIBRARIES}
)

target_link_libraries(scan_to_pcl#
  ${PCL_LIBRARIES}
)

target_link_libraries(path_planner#
  ${PCL_LIBRARIES}
)

# target_link_libraries(odometry
#   tf2_ros::tf2_ros
#   geometry_msgs::geometry_msgs
# )

# link_directories(#
#   ${PCL_LIBRARY_DIRS}
# )

#add_definitions(${PCL_DEFINITIONS})#

# include_directories(#
#   include
#   ${PCL_INCLUDE_DIRS}
#   ${lifecycle_msgs_INCLUDE_DIRS}
#   ${rclcpp_lifecycle_INCLUDE_DIRS}
#   ${rclcpp_INCLUDE_DIRS}
# )


install(TARGETS

  ## 과제
  basic_publisher
  basic_subscriber
  basic_imu
  basic_motor
  basic_pcl
  basic_scan

  ## 중간고사
  m_print_disatnce
  m_put_lidar_angles
  m_put_velocity
  m_main_control

  ## 기말고사
  localization
  scan_to_pcl
  path_planner
  path_tracker

  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/listener_py.py
  
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
