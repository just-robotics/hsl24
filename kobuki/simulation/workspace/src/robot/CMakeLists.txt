cmake_minimum_required(VERSION 3.5)
project(robot)

set(CMAKE_C_STANDARD 20)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config description_master description_slave launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
