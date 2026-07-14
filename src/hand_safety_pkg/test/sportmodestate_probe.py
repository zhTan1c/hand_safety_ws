#!/usr/bin/env python3
"""Interactive recorder for Unitree G1 /sportmodestate posture experiments."""

import argparse
import csv
import datetime as _dt
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from unitree_hg.msg import SportModeState


LABEL_KEYS = {
    "0": "unknown",
    "1": "stand",
    "2": "manual_squat",
    "3": "auto_squat",
    "4": "stand_up",
    "5": "walk",
    "6": "damping",
}


class TerminalMode:
    def __init__(self):
        self.enabled = sys.stdin.isatty()
        self.old_settings = None

    def __enter__(self):
        if self.enabled:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.enabled and self.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def read_key_nonblocking():
    if not sys.stdin.isatty():
        return None
    ready, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not ready:
        return None
    return sys.stdin.read(1)


def fmt_array(values, count=None):
    seq = list(values)
    if count is not None:
        seq = seq[:count]
    return "[" + ", ".join(f"{v:.3f}" if isinstance(v, float) else str(v)
                           for v in seq) + "]"


class SportModeStateProbe(Node):
    def __init__(self, topic, csv_path, print_hz):
        super().__init__("sportmodestate_probe")
        self.topic = topic
        self.csv_path = csv_path
        self.print_period = 1.0 / max(print_hz, 0.1)
        self.label = "unknown"
        self.start_time = time.monotonic()
        self.last_print_time = 0.0
        self.last_msg = None
        self.last_mode = None
        self.sample_count = 0
        self.mode_changes = []
        self.height_stats = {}

        os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)
        self.csv_file = open(csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "time_s",
                "label",
                "mode",
                "progress",
                "gait_type",
                "body_height",
                "velocity_x",
                "velocity_y",
                "velocity_z",
                "yaw_speed",
                "rpy_roll",
                "rpy_pitch",
                "rpy_yaw",
                "foot_force_0",
                "foot_force_1",
                "foot_force_2",
                "foot_force_3",
            ],
        )
        self.writer.writeheader()

        self.sub = self.create_subscription(
            SportModeState, topic, self.on_msg, 10)
        self.get_logger().info(
            f"Recording {topic} to {csv_path}. Press h for help.")

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()

    def set_label(self, label):
        self.label = label
        self.get_logger().warn(f"Label set to: {label}")

    def on_msg(self, msg):
        now = time.monotonic()
        elapsed = now - self.start_time
        self.last_msg = msg
        self.sample_count += 1

        if self.last_mode is None:
            self.last_mode = msg.mode
        elif msg.mode != self.last_mode:
            self.mode_changes.append((elapsed, self.last_mode, msg.mode))
            self.get_logger().warn(
                f"Mode changed: {self.last_mode} -> {msg.mode} at {elapsed:.2f}s")
            self.last_mode = msg.mode

        stat = self.height_stats.setdefault(
            self.label,
            {"min": msg.body_height, "max": msg.body_height, "sum": 0.0, "n": 0},
        )
        stat["min"] = min(stat["min"], msg.body_height)
        stat["max"] = max(stat["max"], msg.body_height)
        stat["sum"] += msg.body_height
        stat["n"] += 1

        row = {
            "time_s": f"{elapsed:.3f}",
            "label": self.label,
            "mode": int(msg.mode),
            "progress": f"{msg.progress:.6f}",
            "gait_type": int(msg.gait_type),
            "body_height": f"{msg.body_height:.6f}",
            "velocity_x": f"{msg.velocity[0]:.6f}",
            "velocity_y": f"{msg.velocity[1]:.6f}",
            "velocity_z": f"{msg.velocity[2]:.6f}",
            "yaw_speed": f"{msg.yaw_speed:.6f}",
            "rpy_roll": f"{msg.imu_state.rpy[0]:.6f}",
            "rpy_pitch": f"{msg.imu_state.rpy[1]:.6f}",
            "rpy_yaw": f"{msg.imu_state.rpy[2]:.6f}",
            "foot_force_0": int(msg.foot_force[0]),
            "foot_force_1": int(msg.foot_force[1]),
            "foot_force_2": int(msg.foot_force[2]),
            "foot_force_3": int(msg.foot_force[3]),
        }
        self.writer.writerow(row)

        if now - self.last_print_time >= self.print_period:
            self.last_print_time = now
            self.print_live(msg, elapsed)

    def print_live(self, msg, elapsed):
        print("\033[2J\033[H", end="")
        print("sportmodestate probe")
        print("====================")
        print(f"topic:       {self.topic}")
        print(f"csv:         {self.csv_path}")
        print(f"time:        {elapsed:.1f}s")
        print(f"samples:     {self.sample_count}")
        print(f"label:       {self.label}")
        print()
        print(f"mode:        {int(msg.mode)}")
        print(f"progress:    {msg.progress:.3f}")
        print(f"gait_type:   {int(msg.gait_type)}")
        print(f"body_height: {msg.body_height:.4f}")
        print(f"velocity:    {fmt_array(msg.velocity)}")
        print(f"yaw_speed:   {msg.yaw_speed:.4f}")
        print(f"rpy:         {fmt_array(msg.imu_state.rpy)}")
        print(f"foot_force:  {fmt_array(msg.foot_force)}")
        print()
        print("labels: 1=stand  2=manual_squat  3=auto_squat")
        print("        4=stand_up  5=walk  6=damping  0=unknown")
        print("keys:   p=print snapshot  h=help  x=exit")
        print()
        self.print_stats()
        sys.stdout.flush()

    def print_stats(self):
        print("body_height stats by label:")
        if not self.height_stats:
            print("  no samples yet")
            return
        for label, stat in sorted(self.height_stats.items()):
            avg = stat["sum"] / max(stat["n"], 1)
            print(
                f"  {label:13s} n={stat['n']:5d} "
                f"min={stat['min']:.4f} avg={avg:.4f} max={stat['max']:.4f}")

    def print_snapshot(self):
        if self.last_msg is None:
            print("No sample received yet.")
            return
        msg = self.last_msg
        elapsed = time.monotonic() - self.start_time
        print()
        print("snapshot")
        print("--------")
        print(f"time_s: {elapsed:.3f}")
        print(f"label: {self.label}")
        print(f"mode: {int(msg.mode)}")
        print(f"progress: {msg.progress:.6f}")
        print(f"gait_type: {int(msg.gait_type)}")
        print(f"body_height: {msg.body_height:.6f}")
        print(f"velocity: {fmt_array(msg.velocity)}")
        print(f"yaw_speed: {msg.yaw_speed:.6f}")
        print(f"rpy: {fmt_array(msg.imu_state.rpy)}")
        print(f"foot_force: {fmt_array(msg.foot_force)}")
        print()


def print_help():
    print()
    print("Experiment steps:")
    print("  1. Press 1 while the robot is standing normally.")
    print("  2. Trigger manual squat, then press 2.")
    print("  3. If testing low-battery automatic squat, press 3 during that state.")
    print("  4. Press 4 while standing up, or 1 after it is stable again.")
    print("  5. Press p any time to print one snapshot.")
    print("  6. Press x to exit; the CSV file is saved continuously.")
    print()


def parse_args():
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    default_csv = f"/tmp/sportmodestate_experiment_{stamp}.csv"
    parser = argparse.ArgumentParser(
        description="Record /sportmodestate while labeling robot posture phases.")
    parser.add_argument("--topic", default="/sportmodestate",
                        help="SportModeState topic to subscribe.")
    parser.add_argument("--csv", default=default_csv,
                        help="Output CSV path.")
    parser.add_argument("--print-hz", type=float, default=5.0,
                        help="Terminal refresh rate.")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = SportModeStateProbe(args.topic, args.csv, args.print_hz)

    try:
        with TerminalMode():
            print_help()
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                key = read_key_nonblocking()
                if key is None:
                    continue
                if key in LABEL_KEYS:
                    node.set_label(LABEL_KEYS[key])
                elif key in ("h", "H", "?"):
                    print_help()
                elif key in ("p", "P"):
                    node.print_snapshot()
                elif key in ("x", "X", "\x03"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f"\nSaved CSV: {args.csv}")


if __name__ == "__main__":
    main()
