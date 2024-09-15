cmake_minimum_required(VERSION 3.5)
project(graph_travel_2)

set(CMAKE_C_STANDARD 20)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(goal_pose_publisher src/goal_pose_publisher.cpp src/points.hpp src/graph.hpp)
ament_target_dependencies(goal_pose_publisher rclcpp geometry_msgs tf2 nav2_msgs)

install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  goal_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
