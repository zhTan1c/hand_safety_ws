# 与 inspire_sdk inspire_hand_touch (sequence<int16,9>、sequence<int16,80>、sequence<int16,96>、sequence<int16,112>) 字段布局一致
# DDS 类型名由 rosidl 生成为 inspire_hand_msgs::msg::dds_::InspireHandTouch_

int16[<=9] fingerone_tip_touch
int16[<=96] fingerone_top_touch
int16[<=80] fingerone_palm_touch
int16[<=9] fingertwo_tip_touch
int16[<=96] fingertwo_top_touch
int16[<=80] fingertwo_palm_touch
int16[<=9] fingerthree_tip_touch
int16[<=96] fingerthree_top_touch
int16[<=80] fingerthree_palm_touch
int16[<=9] fingerfour_tip_touch
int16[<=96] fingerfour_top_touch
int16[<=80] fingerfour_palm_touch
int16[<=9] fingerfive_tip_touch
int16[<=96] fingerfive_top_touch
int16[<=9] fingerfive_middle_touch
int16[<=96] fingerfive_palm_touch
int16[<=112] palm_touch
