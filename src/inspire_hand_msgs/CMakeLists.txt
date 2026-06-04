cmake_minimum_required(VERSION 3.8)
project(inspire_hand_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InspireHandCtrl.msg"
  "msg/InspireHandState.msg"
)

ament_package()
