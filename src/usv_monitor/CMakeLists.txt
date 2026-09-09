cmake_minimum_required(VERSION 3.16)
project(usv_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(usv_monitor_node src/main.cpp)
target_compile_options(usv_monitor_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(usv_monitor_node Qt5::Widgets)
ament_target_dependencies(usv_monitor_node
  rclcpp rcl_interfaces sensor_msgs std_msgs nav_msgs geographic_msgs geometry_msgs)

install(TARGETS usv_monitor_node
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS usv_monitor.sh
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
