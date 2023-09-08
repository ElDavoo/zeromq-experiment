cmake_minimum_required(VERSION 3.5)
project(alfredpennyworth)

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

add_executable(cliente src/cliente.cpp)
ament_target_dependencies(cliente rclcpp std_msgs plotter_time)

add_executable(broker src/broker.cpp)
ament_target_dependencies(broker rclcpp std_msgs plotter_time)

add_executable(worker src/worker.cpp)
ament_target_dependencies(worker rclcpp std_msgs plotter_time)
#ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  cliente
  broker
  worker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
