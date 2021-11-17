cmake_minimum_required(VERSION 3.8)
project(cabin_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cabin_msgs REQUIRED)

add_executable(t4_controller src/t4_controller.cpp)
target_include_directories(t4_controller PUBLIC include)
ament_target_dependencies(t4_controller rclcpp sensor_msgs cabin_msgs)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  t4_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
