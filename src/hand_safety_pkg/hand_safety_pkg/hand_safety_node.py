#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from collections import deque
from inspire_hand_msgs.msg import InspireHandCtrl, InspireHandState
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException


class HandSafetyNode(Node):
    def __init__(self):
        super().__init__('hand_safety_node')

        self.sides = ['l', 'r']
        self.history = {side: deque(maxlen=5) for side in self.sides}
        self.latest_state = {side: None for side in self.sides}
        self.subs = {}
        self.pubs = {}

        # 触发事件发布器：录制节点订阅此话题，在规则触发时记录日志
        self.trigger_pub = self.create_publisher(String, '/safe/inspire_hand/trigger', 10)

        for side in self.sides:
            sub_topic = f'/safe/inspire_hand/raw/cmd/{side}'
            pub_topic = f'/inspire_hand/ctrl/{side}'
            state_topic = f'/inspire_hand/state/{side}'
            
            cb = self.create_callback(side)
            self.subs[side] = self.create_subscription(
                InspireHandCtrl, sub_topic, cb, 10)
            self.pubs[side] = self.create_publisher(
                InspireHandCtrl, pub_topic, 10)
                
            state_cb = self.create_state_callback(side)
            self.create_subscription(
                InspireHandState, state_topic, state_cb, 10)
                
            self.get_logger().info(
                f'Safety node initialized for side {side}: '
                f'{sub_topic} -> {pub_topic}, subscribing state: {state_topic}')

    def create_callback(self, side):
        def callback(msg):
            self.process_msg(msg, side)
        return callback

    def create_state_callback(self, side):
        def callback(msg):
            self.latest_state[side] = msg
        return callback

    def process_msg(self, msg, side):
        hist = self.history[side]
        triggered_rules = []

        # Get angle list from msg
        angle_set = list(msg.angle_set)

        # Get previous frame or fallback
        if len(hist) > 0:
            prev_frame = hist[-1]
        else:
            prev_frame = InspireHandCtrl()
            prev_frame.angle_set = [1000] * 6
            prev_frame.pos_set = [0] * 6
            prev_frame.force_set = [3000] * 6
            prev_frame.speed_set = [1000] * 6
            prev_frame.mode = 13

        # Build list of angle arrays for 5 frames
        frames_angles = [list(f.angle_set) for f in hist] + [list(angle_set)]

        # --- Rule 1 ---
        # （1）连续三帧指令中第4、5、6自由度大于等于0且小于等于200，则其余自由度当前指令不变，
        # 单独将第4、5、6自由度当前的指令定为200
        if len(frames_angles) >= 3:
            last_3_frames = frames_angles[-3:]
            if all(len(f) > 5 for f in last_3_frames):
                if all(0 <= f[3] <= 200 and 0 <= f[4] <= 200 and 0 <= f[5] <= 200 for f in last_3_frames):
                    angle_set[3] = 200
                    angle_set[4] = 200
                    angle_set[5] = 200
                    frames_angles[-1][3] = 200
                    frames_angles[-1][4] = 200
                    frames_angles[-1][5] = 200
                    triggered_rules.append("Rule 1 (joint 4/5/6 <= 200 -> clamp to 200)")

        # --- Rule 2 ---
        # （2）任意自由度受力大于3000，则其余自由度当前指令不变，对超出的自由度用上一帧指令覆盖当前指令
        # 读取 state 里面的 force_act 值进行判断
        state_msg = self.latest_state[side]
        if state_msg is not None and len(state_msg.force_act) > 0:
            forces = state_msg.force_act
        else:
            forces = msg.force_set

        for idx, val in enumerate(forces):
            if val > 3000:
                if len(prev_frame.angle_set) > idx:
                    angle_set[idx] = prev_frame.angle_set[idx]
                if len(frames_angles[-1]) > idx:
                    frames_angles[-1][idx] = angle_set[idx]
                triggered_rules.append(f"Rule 2 (joint {idx} force_act > 3000 overwrite)")

        # --- Rule 3 ---
        # （3）任意自由度指令在0～1000之外，则其余自由度当前指令不变，
        # 小于0的自由度的指令设为0，大于1000的自由度指令设为1000
        for idx, val in enumerate(angle_set):
            if val < 0 or val > 1000:
                if val < 0:
                    angle_set[idx] = 0
                elif val > 1000:
                    angle_set[idx] = 1000
                if len(frames_angles[-1]) > idx:
                    frames_angles[-1][idx] = angle_set[idx]
                triggered_rules.append(f"Rule 3 (joint {idx} angle out of bounds clamp)")

        # --- Rule 4 ---
        # （4）任意自由度连续五帧内，相邻帧之间变化大于200，则其余自由度当前指令不变，
        # 对这个情况的自由度取这五帧指令的平均值赋值到当前指令
        if len(frames_angles) == 5:
            num_dofs = min(len(f) for f in frames_angles)
            for idx in range(num_dofs):
                has_large_change = False
                for k in range(4):
                    if abs(frames_angles[k][idx] - frames_angles[k+1][idx]) > 200:
                        has_large_change = True
                        break
                if has_large_change:
                    avg_val = sum(f[idx] for f in frames_angles) / 5.0
                    angle_set[idx] = int(round(avg_val))
                    triggered_rules.append(f"Rule 4 (joint {idx} diff > 200 avg)")

        # Log triggered rules if any
        if triggered_rules:
            self.get_logger().warn(
                f"[{side}] Safety rule(s) triggered: {', '.join(triggered_rules)}")
            # 发布触发事件，格式: "规则描述:side"
            for rule in triggered_rules:
                trigger_msg = String()
                trigger_msg.data = f'{rule}:{side}'
                self.trigger_pub.publish(trigger_msg)

        # Construct out_msg
        out_msg = InspireHandCtrl()
        out_msg.angle_set = angle_set
        out_msg.pos_set = [0] * 6
        out_msg.force_set = [3000] * 6
        out_msg.speed_set = [1000] * 6
        out_msg.mode = 0b1101

        self.pubs[side].publish(out_msg)
        hist.append(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandSafetyNode()
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
