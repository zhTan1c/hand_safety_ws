#!/usr/bin/env python3
"""
continuous_publish.py
持续发布灵巧手命令到 /safe/inspire_hand/raw/cmd/l 和 r，50Hz。
使用 angle_set 控制，speed 固定 1000，不发 pos_set。

用法:
  终端1: ros2 launch hand_safety_pkg safety_bridge.launch.py
  终端2: python3 test/continuous_publish.py [fps]
"""

import rclpy
from rclpy.node import Node
from inspire_hand_msgs.msg import InspireHandCtrl
import time
import sys


def make_cmd(angle, force=None, mode=0b0001):
    msg = InspireHandCtrl()
    msg.angle_set = [int(v) for v in angle]
    msg.force_set = force if force else [0] * 6
    msg.speed_set = [1000] * 6
    msg.pos_set = [0] * 6
    msg.mode = mode
    return msg


class ContinuousPublisher(Node):
    def __init__(self, fps=50.0):
        super().__init__('continuous_publisher')
        self.fps = fps
        self.dt = 1.0 / fps

        self.pubs = {}
        for side in ['l', 'r']:
            self.pubs[side] = self.create_publisher(
                InspireHandCtrl, f'/safe/inspire_hand/raw/cmd/{side}', 10)

        self.get_logger().info(
            f'Publishing to /safe/inspire_hand/raw/cmd/l and r at {fps} Hz')

    def run(self):
        values = [700] * 6
        count = 0

        self.get_logger().info(f'Warming up with angle={values}...')
        for _ in range(10):
            cmd = make_cmd(values)
            for pub in self.pubs.values():
                pub.publish(cmd)
            time.sleep(self.dt)

        self.get_logger().info('Running. Ctrl+C to stop.')

        try:
            while True:
                start = time.time()

                if (count + 1) % 10 == 0:
                    values = [800] * 6 if values[0] == 700 else [700] * 6
                    self.get_logger().info(f'Switched to angle={values}')

                cmd = make_cmd(values)
                for pub in self.pubs.values():
                    pub.publish(cmd)

                count += 1
                if count % 500 == 0:
                    self.get_logger().info(
                        f'Published {count} frames, angle={values}')

                elapsed = time.time() - start
                time.sleep(max(0, self.dt - elapsed))

        except KeyboardInterrupt:
            self.get_logger().info(f'Stopped after {count} frames.')


def main(args=None):
    fps = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0

    rclpy.init(args=args)
    node = ContinuousPublisher(fps=fps)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
