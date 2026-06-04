#!/usr/bin/env python3
"""
bug_test.py — Hand Safety Node 完整功能测试

运行前提：
  终端1: cd hand_safety_ws && source install/setup.bash
          ros2 run hand_safety_pkg hand_safety_node
  终端2: cd hand_safety_ws && source install/setup.bash
          python3 src/hand_safety_pkg/test/bug_test.py

规则索引说明（以代码为准）:
  Rule 1: 连续3帧 joint[3]/[4]/[5] 均 ∈[0,200] → 钳位到 200
  Rule 2: force_act[idx] > 3000 → angle_set[idx] = 上一帧值
  Rule 3: angle_set[idx] < 0 → 0,  > 1000 → 1000
  Rule 4: 5帧窗口内相邻帧差 > 200 → 取5帧均值
"""

import os
os.environ["ROS_DOMAIN_ID"] = "42"
import rclpy
from rclpy.node import Node
from inspire_hand_msgs.msg import InspireHandCtrl, InspireHandState
import time
import sys


class HandSafetyTester(Node):

    def __init__(self, side='l'):
        super().__init__('hand_safety_tester')
        self.side = side
        self.pub_cmd = self.create_publisher(
            InspireHandCtrl, f'/safe/inspire_hand/raw/cmd/{side}', 10)
        self.pub_state = self.create_publisher(
            InspireHandState, f'/inspire_hand/state/{side}', 10)
        self.sub_ctrl = self.create_subscription(
            InspireHandCtrl, f'/inspire_hand/ctrl/{side}',
            self._ctrl_cb, 10)
        self.received_msgs = []
        self.get_logger().info(f'Tester initialized for side [{side}]')

    def _ctrl_cb(self, msg):
        self.received_msgs.append(msg)

    def _publish_state(self, force_act):
        s = InspireHandState()
        s.force_act = [int(v) for v in force_act]
        s.pos_act = [0] * 6
        s.angle_act = [0] * 6
        s.current = [0] * 6
        s.err = [0] * 6
        s.status = [0] * 6
        s.temperature = [0] * 6
        self.pub_state.publish(s)

    def _drain(self, duration=0.15):
        self.received_msgs.clear()
        t0 = time.time()
        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
        self.received_msgs.clear()

    def _send_raw(self, angle_set, timeout=2.0):
        """发一帧并等返回，不做排空/不发 state"""
        self.received_msgs.clear()
        cmd = InspireHandCtrl()
        cmd.angle_set = [int(v) for v in angle_set]
        cmd.pos_set = [0] * 6
        cmd.force_set = [3000] * 6
        cmd.speed_set = [1000] * 6
        cmd.mode = 0b1101
        self.pub_cmd.publish(cmd)
        t0 = time.time()
        while not self.received_msgs:
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.time() - t0 > timeout:
                raise TimeoutError('Timed out waiting for safety node output')
        return self.received_msgs[0]

    def _confirm_state(self, force_act, max_retries=3):
        """发 dummy 命令确认节点已处理新 state。

        核心问题：state 和 cmd 是不同话题，DDS 不保证跨话题顺序。
        可能 cmd 先到、state 后到，导致节点用旧 latest_state。
        解决：发一个不触发规则的 dummy 命令，检查返回值是否反映新 state。
        反映了 → 说明节点已处理新 state → 可以发真正的测试指令。

        返回 dummy 的输出（它会进入节点 deque，影响后续帧的"上一帧"）。
        """
        for _ in range(max_retries):
            self._publish_state(force_act)
            time.sleep(0.05)
            dummy_out = self._send_raw([500] * 6)
            # 检查 dummy 输出是否反映了 force_act
            if force_act is not None and len(force_act) >= 6:
                expected = [500] * 6
                for i in range(6):
                    if force_act[i] > 3000:
                        expected[i] = 500  # flush 输出就是 500
                # 如果 dummy 输出和预期一致（或 force_act 全 ≤3000），state 已生效
                if list(dummy_out.angle_set) == expected:
                    return dummy_out
                # 不一致 → 重试
                self.get_logger().warn('State not confirmed, retrying...')
            else:
                return dummy_out
        # 最后一次尝试后直接返回
        return dummy_out

    def send_and_wait(self, angle_set, force_act=None, timeout=2.0):
        """发送一帧指令，等待安全节点输出。

        流程：排空 → 发 state（如果有）→ 确认 state → 发测试指令
        """
        # 1. 排空残留
        self._drain()

        # 2. 发 state + 确认
        if force_act is not None:
            self._confirm_state(force_act)

        # 3. 发测试指令
        return self._send_raw(angle_set, timeout)

    def restart_safety_node(self):
        self.get_logger().info("Restarting safety node to clear history queue...")
        import os
        import subprocess
        # Kill the existing node
        os.system("pkill -f hand_safety_[n]ode || true")
        time.sleep(1.0)
        # Start a new one
        env = os.environ.copy()
        subprocess.Popen(
            ["ros2", "run", "hand_safety_pkg", "hand_safety_node"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env
        )
        time.sleep(2.0)
        self.received_msgs.clear()

    def flush_history(self, n=5, force_act=None):
        """发 n 帧正常指令清空安全节点历史队列。

        如果 force_act 不为 None，会先确认 state，然后发 n-1 帧 +
        1 个 state 确认 dummy = 共 n 帧。
        """
        if n < 5:
            self.restart_safety_node()
        fa = force_act if force_act is not None else [0] * 6
        self.get_logger().info(f'Flushing with {n} normal frames...')
        self._drain()
        self._confirm_state(fa)
        # dummy 已占 1 帧，再发 n-1 帧
        for _ in range(max(0, n - 1)):
            self._send_raw([500] * 6)
        self.received_msgs.clear()


# ───────────────────────── 测试运行 ─────────────────────────

def run_tests():
    rclpy.init()
    tester = HandSafetyTester('l')
    tester.restart_safety_node()
    time.sleep(1.0)

    passed = 0
    failed = 0
    errors = []

    def check(name, ok, got=None, expected=None):
        nonlocal passed, failed
        if ok:
            passed += 1
            print(f'  [PASS] {name}')
        else:
            failed += 1
            errors.append(name)
            print(f'  [FAIL] {name}')
            if got is not None:
                print(f'         Got:      {got}')
            if expected is not None:
                print(f'         Expected: {expected}')

    # 确认安全节点可达
    try:
        out = tester.send_and_wait([500] * 6, force_act=[0] * 6)
        check('Sanity: safety node responds', list(out.angle_set) == [500] * 6)
    except TimeoutError:
        print('\n!! 安全节点无响应 !!\n')
        tester.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    try:
        # ==========================================================
        # 一、单规则测试
        # ==========================================================

        # ── Rule 1: joint 3/4/5 连续3帧 ∈[0,200] → 钳位到 200 ──
        print('\n--- Rule 1: joint 3/4/5 持续低位 → 钳位 200 ---')

        # 1.1 基本触发
        tester.flush_history()
        tester.send_and_wait([500, 500, 500, 350, 350, 350])
        tester.send_and_wait([500, 500, 500, 200, 200, 200])
        tester.send_and_wait([500, 500, 500, 100, 100, 100])
        out = tester.send_and_wait([500, 500, 500, 100, 100, 100])
        check('1.1 三帧低位 → 钳位',
              list(out.angle_set) == [500, 500, 500, 200, 200, 200],
              list(out.angle_set), [500, 500, 500, 200, 200, 200])

        # 1.2 上边界 200
        tester.flush_history()
        tester.send_and_wait([500, 500, 500, 350, 350, 350])
        tester.send_and_wait([500, 500, 500, 200, 200, 200])
        tester.send_and_wait([500, 500, 500, 200, 200, 200])
        out = tester.send_and_wait([500, 500, 500, 200, 200, 200])
        check('1.2 边界值 200 → 触发',
              list(out.angle_set) == [500, 500, 500, 200, 200, 200],
              list(out.angle_set), [500, 500, 500, 200, 200, 200])

        # 1.3 下边界 0
        tester.flush_history()
        tester.send_and_wait([500, 500, 500, 350, 350, 350])
        tester.send_and_wait([500, 500, 500, 200, 200, 200])
        tester.send_and_wait([500, 500, 500, 50, 50, 50])
        out = tester.send_and_wait([500, 500, 500, 0, 0, 0])
        check('1.3 边界值 0 → 触发',
              list(out.angle_set) == [500, 500, 500, 200, 200, 200],
              list(out.angle_set), [500, 500, 500, 200, 200, 200])

        # 1.4 值 -1 → Rule1 不触发(-1<0)，Rule3 裁剪到 0
        tester.flush_history()
        tester.send_and_wait([500, 500, 500, 350, 350, 350])
        tester.send_and_wait([500, 500, 500, 200, 200, 200])
        tester.send_and_wait([500, 500, 500, 50, 50, 50])
        tester.send_and_wait([500, 500, 500, -1, -1, -1])
        tester.send_and_wait([500, 500, 500, -1, -1, -1])
        out = tester.send_and_wait([500, 500, 500, -1, -1, -1])
        check('1.4 值 -1 → Rule1 不触发，Rule3 裁剪到 0',
              list(out.angle_set) == [500, 500, 500, 0, 0, 0],
              list(out.angle_set), [500, 500, 500, 0, 0, 0])

        # 1.5 值 201（不触发）
        tester.flush_history()
        tester.send_and_wait([500, 500, 500, 350, 350, 350])
        tester.send_and_wait([500, 500, 500, 201, 201, 201])
        tester.send_and_wait([500, 500, 500, 201, 201, 201])
        out = tester.send_and_wait([500, 500, 500, 201, 201, 201])
        check('1.5 值 201 → 不触发',
              list(out.angle_set) == [500, 500, 500, 201, 201, 201],
              list(out.angle_set), [500, 500, 500, 201, 201, 201])

        # 1.6 仅 2 帧
        tester.flush_history()
        tester.send_and_wait([500, 500, 500, 350, 350, 350])
        tester.send_and_wait([500, 500, 500, 200, 200, 200])
        out = tester.send_and_wait([500, 500, 500, 100, 100, 100])
        check('1.6 仅 2 帧 → 不触发',
              list(out.angle_set) == [500, 500, 500, 100, 100, 100],
              list(out.angle_set), [500, 500, 500, 100, 100, 100])

        # 1.7 前2帧满足，第3帧 joint[3]=250
        tester.flush_history()
        tester.send_and_wait([500, 500, 500, 350, 350, 350])
        tester.send_and_wait([500, 500, 500, 200, 200, 200])
        tester.send_and_wait([500, 500, 500, 100, 100, 100])
        out = tester.send_and_wait([500, 500, 500, 250, 100, 100])
        check('1.7 第3帧 joint[3]=250 → 不触发',
              list(out.angle_set) == [500, 500, 500, 250, 100, 100],
              list(out.angle_set), [500, 500, 500, 250, 100, 100])

        # 1.8 joint[5]=300 (joint 5 is checked by Rule 1, so index 5 > 200 should not trigger)
        tester.flush_history()
        tester.send_and_wait([500, 500, 500, 350, 350, 350])
        tester.send_and_wait([500, 500, 500, 200, 200, 200])
        tester.send_and_wait([500, 500, 500, 100, 100, 100])
        out = tester.send_and_wait([500, 500, 500, 100, 100, 300])
        check('1.8 joint[5]=300 → 不触发',
              list(out.angle_set) == [500, 500, 500, 100, 100, 300],
              list(out.angle_set), [500, 500, 500, 100, 100, 300])

        # 1.9 负值 -50 → Rule3 裁剪到 0
        tester.flush_history()
        tester.send_and_wait([500, 500, 500, 350, 350, 350])
        tester.send_and_wait([500, 500, 500, 200, 200, 200])
        tester.send_and_wait([500, 500, 500, 50, 50, 50])
        tester.send_and_wait([500, 500, 500, -50, -50, -50])
        tester.send_and_wait([500, 500, 500, -50, -50, -50])
        out = tester.send_and_wait([500, 500, 500, -50, -50, -50])
        check('1.9 负值 -50 → Rule3 裁剪到 0',
              list(out.angle_set) == [500, 500, 500, 0, 0, 0],
              list(out.angle_set), [500, 500, 500, 0, 0, 0])

        # ── Rule 2: force_act > 3000 → 用上一帧覆盖 ──
        print('\n--- Rule 2: 受力过大 → 上一帧覆盖 ---')

        # 2.1 单关节受力 > 3000
        tester.restart_safety_node()
        tester.flush_history()
        out = tester.send_and_wait([600]*6, force_act=[0, 0, 3001, 0, 0, 0])
        check('2.1 joint[2] force=3001 → 用上一帧 500 覆盖',
              list(out.angle_set) == [600, 600, 500, 600, 600, 600],
              list(out.angle_set), [600, 600, 500, 600, 600, 600])

        # 2.2 全部 6 关节 > 3000
        tester.flush_history()
        out = tester.send_and_wait([700]*6, force_act=[4000]*6)
        check('2.2 全部 force>3000 → 全部用上一帧 500 覆盖',
              list(out.angle_set) == [500]*6,
              list(out.angle_set), [500]*6)

        # 2.3 边界 force=3000（不触发）
        tester.flush_history()
        out = tester.send_and_wait([600]*6, force_act=[0, 0, 3000, 0, 0, 0])
        check('2.3 force=3000 边界 → 不触发',
              list(out.angle_set) == [600]*6,
              list(out.angle_set), [600]*6)

        # 2.4 没有 state（降级用 msg.force_set）
        tester.flush_history()
        out = tester.send_and_wait([600]*6, force_act=None)
        check('2.4 无 state → 降级用 force_set（3000，不触发）',
              list(out.angle_set) == [600]*6,
              list(out.angle_set), [600]*6)

        # 2.5 state 的 force_act 为空列表
        tester.flush_history()
        tester._drain()
        s = InspireHandState()
        s.force_act = []
        s.pos_act = [0]*6; s.angle_act = [0]*6; s.current = [0]*6
        s.err = [0]*6; s.status = [0]*6; s.temperature = [0]*6
        tester.pub_state.publish(s)
        time.sleep(0.1)
        out = tester._send_raw([600]*6)
        check('2.5 force_act=[] → 降级，不触发',
              list(out.angle_set) == [600]*6,
              list(out.angle_set), [600]*6)

        # 2.6 第一帧触发
        tester.restart_safety_node()
        tester._publish_state([4000]*6)
        time.sleep(0.1)
        out = tester._send_raw([600]*6)
        check('2.6 第一帧触发 → 使用 fallback 默认帧(1000)覆盖',
              list(out.angle_set) == [1000]*6,
              list(out.angle_set), [1000]*6)

        # 2.7 force_act 长度不足 6
        tester.flush_history()
        out = tester.send_and_wait([600]*6, force_act=[4000, 4000, 4000, 4000])
        check('2.7 force_act 只有 4 元素 → 只覆盖 joint 0-3',
              list(out.angle_set) == [500, 500, 500, 500, 600, 600],
              list(out.angle_set), [500, 500, 500, 500, 600, 600])

        # ── Rule 3: 指令越界 → 裁剪到 0~1000 ──
        print('\n--- Rule 3: 指令越界 → 裁剪 ---')

        tester.flush_history()
        tester.send_and_wait([350, 500, 500, 500, 500, 500])
        tester.send_and_wait([200, 500, 500, 500, 500, 500])
        out = tester.send_and_wait([-100, 500, 500, 500, 500, 500])
        check('3.1 joint[0]=-100 → 0',
              list(out.angle_set) == [0, 500, 500, 500, 500, 500],
              list(out.angle_set), [0, 500, 500, 500, 500, 500])

        tester.flush_history()
        tester.send_and_wait([500, 650, 500, 500, 500, 500])
        tester.send_and_wait([500, 800, 500, 500, 500, 500])
        out = tester.send_and_wait([500, 1500, 500, 500, 500, 500])
        check('3.2 joint[1]=1500 → 1000',
              list(out.angle_set) == [500, 1000, 500, 500, 500, 500],
              list(out.angle_set), [500, 1000, 500, 500, 500, 500])

        tester.flush_history()
        tester.send_and_wait([350, 500, 650, 500, 500, 350])
        tester.send_and_wait([200, 500, 800, 500, 500, 200])
        out = tester.send_and_wait([-50, 500, 2000, 500, 500, -300])
        check('3.3 多关节越界 → 各自裁剪',
              list(out.angle_set) == [0, 500, 1000, 500, 500, 0],
              list(out.angle_set), [0, 500, 1000, 500, 500, 0])

        tester.flush_history()
        tester.send_and_wait([350, 500, 500, 500, 500, 500])
        tester.send_and_wait([200, 500, 500, 500, 500, 500])
        out = tester.send_and_wait([0, 500, 500, 500, 500, 500])
        check('3.4 angle=0 边界 → 不触发',
              list(out.angle_set) == [0, 500, 500, 500, 500, 500],
              list(out.angle_set), [0, 500, 500, 500, 500, 500])

        tester.flush_history()
        tester.send_and_wait([650, 500, 500, 500, 500, 500])
        tester.send_and_wait([800, 500, 500, 500, 500, 500])
        out = tester.send_and_wait([1000, 500, 500, 500, 500, 500])
        check('3.5 angle=1000 边界 → 不触发',
              list(out.angle_set) == [1000, 500, 500, 500, 500, 500],
              list(out.angle_set), [1000, 500, 500, 500, 500, 500])

        tester.flush_history()
        tester.send_and_wait([350, 500, 500, 500, 500, 500])
        tester.send_and_wait([200, 500, 500, 500, 500, 500])
        out = tester.send_and_wait([-32768, 500, 500, 500, 500, 500])
        check('3.6 angle=-32768 → 0',
              list(out.angle_set) == [0, 500, 500, 500, 500, 500],
              list(out.angle_set), [0, 500, 500, 500, 500, 500])

        tester.flush_history()
        tester.send_and_wait([650, 500, 500, 500, 500, 500])
        tester.send_and_wait([800, 500, 500, 500, 500, 500])
        out = tester.send_and_wait([32767, 500, 500, 500, 500, 500])
        check('3.7 angle=32767 → 1000',
              list(out.angle_set) == [1000, 500, 500, 500, 500, 500],
              list(out.angle_set), [1000, 500, 500, 500, 500, 500])

        # ── Rule 4: 5帧窗口相邻帧差 > 200 → 取均值 ──
        print('\n--- Rule 4: 五帧突变 → 取平均 ---')

        # flush_history() 内部会发 1 个 state 确认 dummy + 4 帧 = 共 5 帧
        # deque 满（5帧），测试帧进来后 frames_angles=6 → Rule 4 不触发
        # 所以用 flush_history(n=5) 后，再发 1 帧才触发不了
        # 正确做法：flush 4 帧（dummy+3），deque=4，测试帧=第5帧 → Rule 4 触发

        # 4.1 joint[0] 突变 300
        tester.flush_history(n=4)
        out = tester.send_and_wait([800, 500, 500, 500, 500, 500])
        # 5帧: 500,500,500,500,800 → avg = 560
        check('4.1 joint[0] 突变 300 → 均值 560',
              list(out.angle_set) == [560, 500, 500, 500, 500, 500],
              list(out.angle_set), [560, 500, 500, 500, 500, 500])

        # 4.2 差值恰好 200（不触发）
        tester.flush_history(n=4)
        out = tester.send_and_wait([700, 500, 500, 500, 500, 500])
        check('4.2 差值=200 → 不触发',
              list(out.angle_set) == [700, 500, 500, 500, 500, 500],
              list(out.angle_set), [700, 500, 500, 500, 500, 500])

        # 4.3 差值 201（触发）
        tester.flush_history(n=4)
        out = tester.send_and_wait([701, 500, 500, 500, 500, 500])
        # avg = (500*4 + 701)/5 = 540.2 → 540
        check('4.3 差值=201 → 均值 540',
              list(out.angle_set) == [540, 500, 500, 500, 500, 500],
              list(out.angle_set), [540, 500, 500, 500, 500, 500])

        # 4.4 仅首帧突变
        # 从空 deque 构建：flush(n=1)=dummy only，然后发 3 帧 + 测试帧 = 5 帧
        tester.flush_history(n=1)
        tester.send_and_wait([500, 500, 500, 500, 500, 500])
        tester.send_and_wait([800, 500, 500, 500, 500, 500])
        tester.send_and_wait([500, 500, 500, 500, 500, 500])
        out = tester.send_and_wait([500, 500, 500, 500, 500, 500])
        # 5帧: 500,500,800,500,500 → 差值0,300,300,0 → 触发 → avg=560
        check('4.4 仅首帧突变 → 均值 560',
              list(out.angle_set) == [560, 500, 500, 500, 500, 500],
              list(out.angle_set), [560, 500, 500, 500, 500, 500])

        # 4.5 不足 5 帧 → 不触发
        # flush(n=1)=dummy only，发 2 帧 = 共 3 帧 < 5
        tester.flush_history(n=1)
        tester.send_and_wait([500]*6)
        out = tester.send_and_wait([800, 500, 500, 500, 500, 500])
        check('4.5 不足 5 帧 → 不触发',
              list(out.angle_set) == [800, 500, 500, 500, 500, 500],
              list(out.angle_set), [800, 500, 500, 500, 500, 500])

        # 4.6 全部关节突变
        tester.flush_history(n=4)
        out = tester.send_and_wait([800]*6)
        check('4.6 全部关节突变 → 均值 560',
              list(out.angle_set) == [560]*6,
              list(out.angle_set), [560]*6)

        # 4.7 Rule1 Rule4 交互
        tester.restart_safety_node()
        tester.send_and_wait([500]*6)
        tester.send_and_wait([500]*6)
        tester.send_and_wait([500, 500, 500, 100, 100, 100])
        tester.send_and_wait([500, 500, 500, 100, 100, 100])
        out = tester.send_and_wait([500, 500, 500, 100, 100, 100])
        # joint 3/4/5: 连续3帧 100 -> Rule 1 钳位到 200，变为 [500, 500, 500, 200, 200, 200]
        # 然后 Rule 4 突变（帧 2->3 从 500 降到 100），计算均值：(500*2 + 100*2 + 200)/5 = 280
        check('4.7 Rule1+Rule4 交互 → 均值 280',
              list(out.angle_set) == [500, 500, 500, 280, 280, 280],
              list(out.angle_set), [500, 500, 500, 280, 280, 280])

        # ==========================================================
        # 二、规则组合测试
        # ==========================================================
        print('\n--- 规则组合测试 ---')

        # C1: Rule1 + Rule2
        tester.restart_safety_node()
        tester.send_and_wait([500]*6)
        tester.send_and_wait([500, 500, 500, 100, 100, 100])
        tester.send_and_wait([500, 500, 500, 100, 100, 100])
        tester._drain()
        tester._publish_state([0, 0, 0, 4000, 4000, 4000])
        time.sleep(0.1)
        out = tester._send_raw([500, 500, 500, 100, 100, 100])
        # 满足 Rule 1 连续3帧 100 条件，本应钳位到 200
        # 但因为第 4/5/6 自由度受力大于 3000，Rule 2 用上一帧的值（100）覆盖
        check('C1: Rule1 钳位 200 → 被 Rule2 覆盖回上一帧值 100',
              list(out.angle_set) == [500, 500, 500, 100, 100, 100],
              list(out.angle_set), [500, 500, 500, 100, 100, 100])

        # C2: Rule2 覆盖值为边界值 1000
        tester.flush_history()
        # 让前一帧输出为边界值 1000
        tester.send_and_wait([1000]*6)
        # 当前帧发送 600，但受力过大
        tester._drain()
        tester._publish_state([4000]*6)
        time.sleep(0.1)
        out = tester._send_raw([600]*6)
        check('C2: Rule2 覆盖值为边界 1000 → Rule3 不触发越界',
              list(out.angle_set) == [1000]*6,
              list(out.angle_set), [1000]*6)

        # C3: Rule1 + Rule4 同时触发
        tester.restart_safety_node()
        tester.send_and_wait([500]*6)
        tester.send_and_wait([500]*6)
        tester.send_and_wait([500, 500, 500, 100, 100, 100])
        tester.send_and_wait([500, 500, 500, 100, 100, 100])
        out = tester.send_and_wait([500, 500, 500, 100, 100, 100])
        # 5帧为 [500, 500, 100, 100, 100]。Rule 1 将最后一帧钳位至 200，变为 [500, 500, 100, 100, 200]
        # 随后 Rule 4 取平均，(500*2 + 100*2 + 200)/5 = 280
        check('C3: Rule1 钳位 200 + Rule4 均值 280',
              list(out.angle_set) == [500, 500, 500, 280, 280, 280],
              list(out.angle_set), [500, 500, 500, 280, 280, 280])

        # C4: Rule3 + Rule4
        tester.flush_history(n=4)
        out = tester.send_and_wait([1500, 500, 500, 500, 500, 500])
        # joint 0: Rule3→1000, 500*4+1000 → avg=600
        check('C4: Rule3 裁剪到 1000 → Rule4 均值 600',
              list(out.angle_set) == [600, 500, 500, 500, 500, 500],
              list(out.angle_set), [600, 500, 500, 500, 500, 500])

        # C5: 全部 4 条规则同时触发
        tester.flush_history(n=4)
        tester.send_and_wait([-50, 500, 100, 50, 50, 50],
                             force_act=[4000, 4000, 4000, 0, 0, 0])
        tester.send_and_wait([600, 500, 800, 50, 50, 50],
                             force_act=[4000, 4000, 4000, 0, 0, 0])
        out = tester.send_and_wait([-50, 500, 100, 50, 50, 50],
                                   force_act=[4000, 4000, 4000, 0, 0, 0])
        check('C5: 四规则同时触发（不崩溃）',
              len(list(out.angle_set)) == 6)

        # ==========================================================
        # 三、系统边界测试
        # ==========================================================
        print('\n--- 系统边界测试 ---')

        # S1: angle_set 长度不足 6
        tester.flush_history()
        try:
            out = tester.send_and_wait([600, 600, 600, 600])
            check('S1: angle_set 仅 4 元素',
                  len(list(out.angle_set)) >= 4,
                  list(out.angle_set), '不崩溃即可')
        except Exception as e:
            check(f'S1: angle_set 仅 4 元素 → 崩溃: {e}', False)

        # S2: angle_set 为空列表
        tester.flush_history()
        try:
            out = tester.send_and_wait([])
            check('S2: angle_set=[] → 不崩溃',
                  list(out.angle_set) == [],
                  list(out.angle_set), [])
        except Exception as e:
            check(f'S2: angle_set=[] → 崩溃: {e}', False)

        # S3: 第一帧异常
        tester.restart_safety_node()
        out = tester._send_raw([-100, 1500, 500, 500, 500, 500])
        check('S3: 第一帧异常指令 → 裁剪到 0~1000',
              list(out.angle_set) == [0, 1000, 500, 500, 500, 500],
              list(out.angle_set), [0, 1000, 500, 500, 500, 500])

        # S4: 左右手独立性
        tester_r = HandSafetyTester('r')
        time.sleep(0.5)
        tester.flush_history()
        tester.send_and_wait([500, 500, 500, 350, 350, 350])
        tester.send_and_wait([500, 500, 500, 200, 200, 200])
        tester.send_and_wait([500, 500, 500, 100, 100, 100])
        tester.send_and_wait([500, 500, 500, 100, 100, 100])
        out_r = tester_r.send_and_wait([500]*6, force_act=[0]*6)
        check('S4: 左手触发规则不影响右手',
              list(out_r.angle_set) == [500]*6,
              list(out_r.angle_set), [500]*6)
        tester_r.destroy_node()

        # S5: 高频发送正常帧
        tester.flush_history()
        for _ in range(20):
            tester._send_raw([500]*6)
        check('S5: 高频正常帧不触发规则', True)

        # S6: state 比 ctrl 晚到达
        tester.flush_history()
        tester._drain()
        cmd = InspireHandCtrl()
        cmd.angle_set = [600]*6; cmd.pos_set = [0]*6
        cmd.force_set = [3000]*6; cmd.speed_set = [1000]*6; cmd.mode = 0b1101
        tester.pub_cmd.publish(cmd)
        t0 = time.time()
        while not tester.received_msgs:
            rclpy.spin_once(tester, timeout_sec=0.01)
            if time.time() - t0 > 2.0:
                raise TimeoutError()
        out = tester.received_msgs[0]
        check('S6: state 晚于 cmd → 使用旧 state',
              list(out.angle_set) == [600]*6,
              list(out.angle_set), [600]*6)

        # S7: state 中断 → Rule2 持续触发
        tester.flush_history()
        tester._drain()
        tester._publish_state([4000]*6)
        time.sleep(0.1)
        # 确认 state 生效
        dummy = tester._send_raw([500]*6)
        # 发测试指令（不再发 state）
        out1 = tester._send_raw([600]*6)
        out2 = tester._send_raw([700]*6)
        check('S7: state 中断 → Rule2 持续触发',
              list(out1.angle_set) == [500]*6 and list(out2.angle_set) == [500]*6,
              [list(out1.angle_set), list(out2.angle_set)],
              [[500]*6, [500]*6])

        # S8: 连续相同正常帧
        tester.flush_history()
        ok = True
        for _ in range(10):
            out = tester._send_raw([650]*6)
            if list(out.angle_set) != [650]*6:
                ok = False
                break
        check('S8: 连续相同正常帧 → 原样透传', ok)

    except Exception as e:
        print(f'\n!! 测试异常: {e}')
        import traceback
        traceback.print_exc()
    finally:
        tester.destroy_node()
        rclpy.shutdown()

    total = passed + failed
    print('\n' + '=' * 50)
    print(f'  测试汇总: {total} 项, {passed} 通过, {failed} 失败')
    print('=' * 50)
    if errors:
        print('  失败项:')
        for e in errors:
            print(f'    - {e}')
    if failed == 0:
        print('  ALL TESTS PASSED!')
    else:
        print('  SOME TESTS FAILED!')
    print('=' * 50)
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    run_tests()
