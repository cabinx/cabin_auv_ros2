cmake_minimum_required(VERSION 3.8)
project(cabin_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cabin_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Ceres REQUIRED)

add_executable(thruster_controller src/thruster_controller.cpp)
target_include_directories(thruster_controller PUBLIC include)
target_link_libraries(thruster_controller yaml-cpp ${CERES_LIBRARIES})
ament_target_dependencies(thruster_controller rclcpp cabin_msgs geometry_msgs)

add_executable(pwm_controller src/pwm_controller.cpp)
target_include_directories(pwm_controller PUBLIC include)
target_link_libraries(pwm_controller yaml-cpp)
ament_target_dependencies(pwm_controller rclcpp cabin_msgs std_msgs)


install(TARGETS
  thruster_controller
  pwm_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  cfg
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
