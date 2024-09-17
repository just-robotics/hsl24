cmake_minimum_required(VERSION 3.8)
project(dynamic_nav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/CostmapCircle.msg"
    "msg/DetectorData.msg"
    "msg/DynamicLayerMsg.msg"
    "msg/ObstaclesFootprints.msg"
    "msg/YoloData.msg"
    "msg/MinDistance.msg"
    DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
