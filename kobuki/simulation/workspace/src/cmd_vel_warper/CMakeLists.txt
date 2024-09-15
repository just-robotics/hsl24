cmake_minimum_required(VERSION 3.5)
project(cmd_vel_warper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(warper src/warper.cpp)
ament_target_dependencies(warper ${dependencies})

install(TARGETS
    warper
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
