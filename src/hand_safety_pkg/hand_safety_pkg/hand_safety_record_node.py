#!/usr/bin/env python3
"""
hand_safety_record_node.py — 安全规则触发日志录制节点

功能：
  1. 订阅灵巧手的输入/输出指令话题，用 maxlen=10 的 deque 滑动窗口持续记录
  2. 订阅安全节点的触发话题 /hand_safety/trigger
  3. 触发时，把最近 5 帧 × 2 话题 × 2 手 = 20 帧数据写入 .log 文件

日志路径：~/safety_logs/safety_trigger_YYYYMMDD_HHMMSS.log
每个触发事件一个独立文件，方便检查和维护。

用法：
  ros2 run hand_safety_pkg hand_safety_record_node
  ros2 run hand_safety_pkg hand_safety_record_node --ros-args -p log_dir:=/tmp/my_logs

话题：
  订阅:
    /safe/inspire_hand/raw/cmd/l  (InspireHandCtrl)  — 安全节点输入，左手
    /safe/inspire_hand/raw/cmd/r  (InspireHandCtrl)  — 安全节点输入，右手
    /inspire_hand/ctrl/l          (InspireHandCtrl)  — 安全节点输出，左手
    /inspire_hand/ctrl/r          (InspireHandCtrl)  — 安全节点输出，右手
    /safe/inspire_hand/trigger    (String)           — 规则触发事件
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from collections import deque
from datetime import datetime
from inspire_hand_msgs.msg import InspireHandCtrl
from std_msgs.msg import String


class HandSafetyRecordNode(Node):

    def __init__(self):
        super().__init__('hand_safety_record_node')

        # 参数：日志目录
        self.declare_parameter('log_dir', os.path.expanduser('~/safety_logs'))
        log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        os.makedirs(log_dir, exist_ok=True)
        self.log_dir = log_dir

        # 滑动窗口：每个话题记录最近 10 帧
        self.window_size = 10
        self.dump_frames = 5  # 触发时写入的帧数

        # key = topic 名，value = deque of (timestamp_sec, InspireHandCtrl)
        self.frames = {}

        # 订阅 4 个数据话题
        topics = {
            'raw_cmd_l': '/safe/inspire_hand/raw/cmd/l',
            'raw_cmd_r': '/safe/inspire_hand/raw/cmd/r',
            'ctrl_l':    '/inspire_hand/ctrl/l',
            'ctrl_r':    '/inspire_hand/ctrl/r',
        }
        self.topic_subs = {}
        for key, topic in topics.items():
            self.frames[key] = deque(maxlen=self.window_size)
            # 用闭包捕获 key
            def make_cb(k):
                def cb(msg):
                    ts = self.get_clock().now().nanoseconds / 1e9
                    self.frames[k].append((ts, msg))
                return cb
            self.topic_subs[key] = self.create_subscription(
                InspireHandCtrl, topic, make_cb(key), 10)

        # 订阅触发话题
        self.trigger_sub = self.create_subscription(
            String, '/safe/inspire_hand/trigger', self._on_trigger, 10)

        self.trigger_count = 0
        self.get_logger().info(
            f'Record node started. Logging to: {self.log_dir}')
        self.get_logger().info(
            f'Subscribed to: {list(topics.values())} + /safe/inspire_hand/trigger')

    def _on_trigger(self, msg):
        """收到安全节点的规则触发事件"""
        self.trigger_count += 1
        now = datetime.now()
        trigger_data = msg.data  # e.g. "Rule 1 (joint 4/5/6 <= 200 -> clamp to 200):l"

        # 解析触发信息
        parts = trigger_data.rsplit(':', 1)
        rule_desc = parts[0] if len(parts) == 2 else trigger_data
        side = parts[1] if len(parts) == 2 else '?'

        filename = f'safety_trigger_{now.strftime("%Y%m%d_%H%M%S")}_{self.trigger_count:04d}.log'
        filepath = os.path.join(self.log_dir, filename)

        self.get_logger().warn(
            f'Trigger #{self.trigger_count}: {trigger_data} -> {filename}')

        self._dump_frames(filepath, rule_desc, side, now)

    def _dump_frames(self, filepath, rule_desc, side, now):
        """把最近 5 帧数据写入日志文件"""
        with open(filepath, 'w') as f:
            # ── 头部 ──
            f.write('=' * 70 + '\n')
            f.write('  HAND SAFETY TRIGGER LOG\n')
            f.write('=' * 70 + '\n')
            f.write(f'  Time:     {now.strftime("%Y-%m-%d %H:%M:%S")}\n')
            f.write(f'  Rule:     {rule_desc}\n')
            f.write(f'  Side:     {side}\n')
            f.write(f'  Window:   {self.dump_frames} frames per topic\n')
            f.write('=' * 70 + '\n\n')

            # ── 数据 ──
            # 按 [左手输入, 左手输出, 右手输入, 右手输出] 顺序输出
            sections = [
                ('raw_cmd_l', 'LEFT  INPUT  (raw_cmd/l)', 'l'),
                ('ctrl_l',    'LEFT  OUTPUT (ctrl/l)',    'l'),
                ('raw_cmd_r', 'RIGHT INPUT  (raw_cmd/r)', 'r'),
                ('ctrl_r',    'RIGHT OUTPUT (ctrl/r)',    'r'),
            ]

            for key, title, s in sections:
                marker = ' <<<' if s == side else ''
                f.write(f'--- {title}{marker} ---\n')

                buf = list(self.frames[key])
                # 取最后 dump_frames 帧
                recent = buf[-self.dump_frames:] if len(buf) >= self.dump_frames else buf

                if not recent:
                    f.write('  (no data)\n\n')
                    continue

                for i, (ts, frame_msg) in enumerate(recent):
                    angle = list(frame_msg.angle_set)
                    force = list(frame_msg.force_set)
                    pos = list(frame_msg.pos_set)
                    speed = list(frame_msg.speed_set)
                    f.write(f'  Frame {i+1}/{len(recent)} (t={ts:.6f}):\n')
                    f.write(f'    angle_set = {angle}\n')
                    f.write(f'    force_set = {force}\n')
                    f.write(f'    pos_set   = {pos}\n')
                    f.write(f'    speed_set = {speed}\n')
                    f.write(f'    mode      = {frame_msg.mode}\n')

                f.write('\n')

            # ── 对比分析（触发侧）──
            f.write('=' * 70 + '\n')
            f.write(f'  DIFF ANALYSIS (side={side})\n')
            f.write('=' * 70 + '\n')

            input_key = f'raw_cmd_{side}'
            output_key = f'ctrl_{side}'
            input_buf = list(self.frames[input_key])
            output_buf = list(self.frames[output_key])

            if input_buf and output_buf:
                n = min(self.dump_frames, len(input_buf), len(output_buf))
                for i in range(-n, 0):
                    inp = input_buf[i][1]
                    out = output_buf[i][1]
                    in_angle = list(inp.angle_set)
                    out_angle = list(out.angle_set)
                    # 找出被修改的关节
                    changed = [j for j in range(min(len(in_angle), len(out_angle)))
                               if in_angle[j] != out_angle[j]]
                    marker = ' <-- CHANGED' if changed else ''
                    f.write(f'  Frame {-i}/{n}: input={in_angle} -> output={out_angle}{marker}\n')
                    if changed:
                        for j in changed:
                            f.write(f'    joint[{j}]: {in_angle[j]} -> {out_angle[j]}\n')
            else:
                f.write('  (insufficient data for diff)\n')

            f.write('\n' + '=' * 70 + '\n')

        self.get_logger().info(f'Log written: {filepath}')


def main(args=None):
    rclpy.init(args=args)
    node = HandSafetyRecordNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
