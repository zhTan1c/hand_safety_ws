# 与 inspire_sdk inspire_hand_state (sequence<int16,6>、sequence<unit8,6>) 字段布局一致
# DDS 类型名由 rosidl 生成为 inspire_hand_msgs::msg::dds_::InspireHandState_

int16[<=6] pos_act
int16[<=6] angle_act
int16[<=6] force_act
int16[<=6] current
uint8[<=6] err
uint8[<=6] status
uint8[<=6] temperature