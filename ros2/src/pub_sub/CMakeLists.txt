cmake_minimum_required(VERSION 3.5)
project(pub_sub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plotter_time REQUIRED)   
#find_package(rosidl_default_generators REQUIRED)
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/Plotter.msg"
#)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs plotter_time)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs plotter_time)

#ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
