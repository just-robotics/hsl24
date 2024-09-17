cmake_minimum_required(VERSION 3.8)
project(data_synchronizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dynamic_nav_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(data_synchronizer
    src/data_synchronizer.cpp
)

add_executable(fake_realsense
    src/fake_realsense.cpp
)

add_executable(yolo_dispatcher
    src/yolo_dispatcher.cpp
)

target_link_libraries(data_synchronizer ${OpenCV_LIBS})
target_link_libraries(fake_realsense ${OpenCV_LIBS})
target_link_libraries(yolo_dispatcher ${OpenCV_LIBS})

ament_target_dependencies(data_synchronizer dynamic_nav_interfaces message_filters nav_msgs rclcpp sensor_msgs)
ament_target_dependencies(fake_realsense cv_bridge nav_msgs rclcpp sensor_msgs)
ament_target_dependencies(yolo_dispatcher cv_bridge dynamic_nav_interfaces nav_msgs rclcpp sensor_msgs)

install(DIRECTORY
    src config launch
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    data_synchronizer
    fake_realsense
    yolo_dispatcher
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
