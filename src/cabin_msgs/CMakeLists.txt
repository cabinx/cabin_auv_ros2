cmake_minimum_required(VERSION 3.8)
project(cabin_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AttitudeCommand.msg"
  "msg/ControlStatus.msg"
  "msg/Depth.msg"
  "msg/DepthCommand.msg"
  "msg/Imu.msg"
  "msg/LinearCommand.msg"
  "msg/NetLoad.msg"
  "msg/Pwm.msg"
  "msg/PwmStamped.msg"
  "msg/ResetControls.msg"
  "msg/SwitchState.msg"
  "msg/Thrust.msg"
  "msg/ThrustStamped.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
