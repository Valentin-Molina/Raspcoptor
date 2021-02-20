
cmake_minimum_required(VERSION 3.5)
# TODO give a name to your project
project(imu_node)

set(CMAKE_CXX_STANDARD 14)

# TODO find necessary libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)
# add_library(I2Cdev SHARED 
#             include/I2Cdev.h src/I2Cdev.cpp)
# add_library(MPU6050 SHARED 
#             include/MPU6050.h src/MPU6050.cpp)
# target_link_libraries(MPU6050 I2Cdev)

add_executable(imu_node src/imu.cpp
		include/I2Cdev.h src/I2Cdev.cpp
		include/MPU6050.h src/MPU6050.cpp)
# target_link_libraries(imu_node MPU6050)


# TODO include non-ROS2 directories (OpenCV, Yaml-cpp, etc.)
#target_include_directories(IMU PRIVATE ${YamlCpp_INCLUDE_DIRS})

# TODO link your node to its ROS 2 dependencies
ament_target_dependencies(imu_node
  rclcpp
  std_msgs
  sensor_msgs)
  
# TODO let ament create an executable from your node
# rclcpp_components_register_node(IMU
#   PLUGIN "<package name>::<node class name>"
#   EXECUTABLE <executable_name>)

# install things
install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME})
  
# install other directories if needed
# install(DIRECTORY launch urdf maps params
#   DESTINATION share/${PROJECT_NAME})

# register node
# ament_export_libraries(IMU)
ament_package()
