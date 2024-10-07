cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(master_controller
    src/master_controller.cpp
)

add_executable(dispatcher
    src/dispatcher.cpp
)

ament_target_dependencies(master_controller
    geometry_msgs
    nav2_msgs
    rclcpp
    rclcpp_action
    std_msgs
    nav_msgs
    tf2
)

ament_target_dependencies(dispatcher
    geometry_msgs
    rclcpp
    std_msgs
    tf2
)

install(DIRECTORY
  config launch src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  master_controller
  dispatcher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
