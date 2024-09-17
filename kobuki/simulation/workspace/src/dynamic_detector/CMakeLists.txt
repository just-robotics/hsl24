cmake_minimum_required(VERSION 3.8)
project(dynamic_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dynamic_nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(dynamic_detector
    src/dynamic_detector.cpp
)

add_executable(obstacles_tracker
    src/obstacles_tracker.cpp
)

add_executable(pc2_visualizer
    src/pc2_visualizer.cpp
)

target_link_libraries(dynamic_detector ${OpenCV_LIBS})
target_link_libraries(obstacles_tracker ${OpenCV_LIBS})
target_link_libraries(pc2_visualizer ${OpenCV_LIBS})

ament_target_dependencies(dynamic_detector
    cv_bridge
    dynamic_nav_interfaces
    geometry_msgs
    image_geometry
    rclcpp
    sensor_msgs
    std_msgs
    visualization_msgs
)

ament_target_dependencies(obstacles_tracker
dynamic_nav_interfaces
    geometry_msgs
    nav_msgs
    nav2_msgs
    rclcpp
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

ament_target_dependencies(pc2_visualizer
    cv_bridge
    dynamic_nav_interfaces
    image_geometry
    rclcpp
    sensor_msgs
)

install(DIRECTORY
    src config launch
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    dynamic_detector
    obstacles_tracker
    pc2_visualizer
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
