cmake_minimum_required(VERSION 3.8)
project(odom_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include)

add_executable(odom_calib 
  src/main.cpp 
  src/math.cpp
)

ament_target_dependencies(odom_calib 
  rclcpp 
  rosbag2_cpp 
  rosbag2_storage
  std_msgs 
  geometry_msgs 
  nav_msgs
)

target_include_directories(odom_calib PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS odom_calib
  DESTINATION lib/${PROJECT_NAME})

ament_package()