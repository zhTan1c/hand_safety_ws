cmake_minimum_required(VERSION 3.8)
project(hand_safety_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(inspire_hand_msgs REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)

# ── C++ 安全节点 ──
add_executable(hand_safety_node
  hand_safety_pkg/hand_safety_node.cpp)
ament_target_dependencies(hand_safety_node
  rclcpp std_msgs inspire_hand_msgs)
install(TARGETS hand_safety_node
  DESTINATION lib/${PROJECT_NAME})

# ── C++ 手柄急停节点 ──
add_executable(hand_gamepad_estop
  hand_safety_pkg/hand_gamepad_estop.cpp)
ament_target_dependencies(hand_gamepad_estop
  rclcpp std_msgs unitree_hg)
install(TARGETS hand_gamepad_estop
  DESTINATION lib/${PROJECT_NAME})

# ── Python 节点（录制节点等）──
set(_python_dest "lib/python3/dist-packages/${PROJECT_NAME}")
install(FILES
  hand_safety_pkg/__init__.py
  # hand_safety_pkg/hand_safety_node.py        # 已由 C++ 版替代
  hand_safety_pkg/hand_safety_record_node.py
  hand_safety_pkg/hand_safety_voice_node.py
  DESTINATION ${_python_dest}
)

# ── Python 可执行文件 ──
# install(PROGRAMS
#   hand_safety_pkg/hand_safety_node.py          # 已由 C++ 版替代
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME hand_safety_node
# )
install(PROGRAMS
  hand_safety_pkg/hand_safety_record_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hand_safety_record_node
)
install(PROGRAMS
  hand_safety_pkg/hand_safety_voice_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hand_safety_voice_node
)

ament_export_dependencies(inspire_hand_msgs std_msgs unitree_hg unitree_api)
ament_package()
