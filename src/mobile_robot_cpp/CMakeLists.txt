cmake_minimum_required(VERSION 3.8)
project(mobile_robot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

# executable
add_executable(lcd_display_node src/lcd_display_node.cpp)
ament_target_dependencies(lcd_display_node rclcpp sensor_msgs std_msgs geometry_msgs tf2_ros)

install(TARGETS lcd_display_node
  DESTINATION lib/${PROJECT_NAME}
)

# install other package resources (model, launch, parameters, rviz, config)
install(DIRECTORY launch model parameters rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

