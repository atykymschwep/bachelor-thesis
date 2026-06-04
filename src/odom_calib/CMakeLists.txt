cmake_minimum_required(VERSION 3.8)
project(odom_calib)

# Поиск необходимых пакетов ROS 2 и внешних библиотек
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Включение папки с заголовочными файлами
include_directories(include)

# ==============================================================================
# 1. Таргет для КАЛИБРАЦИИ (Основная программа расчета коэффициентов)
# ==============================================================================
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

# ==============================================================================
# 2. Таргет для ВЕРИФИКАЦИИ (Программа для генерации CSV-файла графиков)
# ==============================================================================
add_executable(odom_verification 
  src/verification.cpp 
  src/math.cpp
)

ament_target_dependencies(odom_verification 
  rclcpp 
  rosbag2_cpp 
  rosbag2_storage
  std_msgs 
  geometry_msgs 
  nav_msgs
)

target_include_directories(odom_verification PRIVATE ${EIGEN3_INCLUDE_DIRS})

# ==============================================================================
# Установка исполняемых файлов в рабочее пространство
# ==============================================================================
install(TARGETS 
  odom_calib
  odom_verification
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()