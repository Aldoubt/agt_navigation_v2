#!/usr/bin/env python3

"""SOFTWARE_ONLY keyboard teleop for the V25-11 Gazebo validation stack.

The node publishes Twist commands to the same post-safety command topic used by
V25-11A. It is intentionally a manual simulation tool and is not part of the
mission / Nav2 command path.
"""

from __future__ import annotations

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
V25-11 Gazebo keyboard teleop (SOFTWARE_ONLY)

  q   w   e        forward-left / forward / forward-right
  a   x   d        rotate-left  / STOP    / rotate-right
  z   s   c        reverse-left / reverse / reverse-right

  SPACE            STOP
  + / -            increase / decrease linear speed
  ] / [            increase / decrease angular speed
  h                 show this help
  Ctrl-C            stop and quit

Hold or repeatedly press a motion key. A deadman watchdog automatically sends
zero velocity when keyboard input stops.
""".strip()


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("agt_gazebo_keyboard_teleop")
        self.command_topic = str(
            self.declare_parameter("command_topic", "/agt/safety/cmd_vel").value
        )
        self.linear_speed = float(self.declare_parameter("linear_speed", 0.25).value)
        self.angular_speed = float(self.declare_parameter("angular_speed", 0.60).value)
        self.linear_step = float(self.declare_parameter("linear_step", 0.05).value)
        self.angular_step = float(self.declare_parameter("angular_step", 0.10).value)
        self.max_linear_speed = float(
            self.declare_parameter("max_linear_speed", 0.60).value
        )
        self.max_angular_speed = float(
            self.declare_parameter("max_angular_speed", 1.20).value
        )
        self.deadman_timeout_s = float(
            self.declare_parameter("deadman_timeout_s", 0.80).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 20.0).value
        )

        if self.linear_speed <= 0.0 or self.angular_speed <= 0.0:
            raise ValueError("linear_speed and angular_speed must be positive")
        if self.deadman_timeout_s <= 0.0 or self.publish_rate_hz <= 0.0:
            raise ValueError("deadman_timeout_s and publish_rate_hz must be positive")

        self.publisher = self.create_publisher(Twist, self.command_topic, 10)
        self.linear = 0.0
        self.angular = 0.0
        self.last_motion_key_wall = 0.0
        self.last_publish_wall = 0.0

    def set_motion(self, linear_scale: float, angular_scale: float) -> None:
        self.linear = linear_scale * self.linear_speed
        self.angular = angular_scale * self.angular_speed
        self.last_motion_key_wall = time.monotonic()
        self.publish_command(force=True)

    def stop(self) -> None:
        self.linear = 0.0
        self.angular = 0.0
        self.last_motion_key_wall = 0.0
        self.publish_command(force=True)

    def adjust_linear(self, delta: float) -> None:
        self.linear_speed = max(
            self.linear_step,
            min(self.max_linear_speed, self.linear_speed + delta),
        )
        print(f"linear_speed={self.linear_speed:.2f} m/s")

    def adjust_angular(self, delta: float) -> None:
        self.angular_speed = max(
            self.angular_step,
            min(self.max_angular_speed, self.angular_speed + delta),
        )
        print(f"angular_speed={self.angular_speed:.2f} rad/s")

    def watchdog(self) -> None:
        if (
            self.last_motion_key_wall > 0.0
            and time.monotonic() - self.last_motion_key_wall > self.deadman_timeout_s
            and (self.linear != 0.0 or self.angular != 0.0)
        ):
            self.stop()

    def publish_command(self, force: bool = False) -> None:
        now = time.monotonic()
        period = 1.0 / self.publish_rate_hz
        if not force and now - self.last_publish_wall < period:
            return
        message = Twist()
        message.linear.x = float(self.linear)
        message.angular.z = float(self.angular)
        self.publisher.publish(message)
        self.last_publish_wall = now


def _read_key(timeout_s: float = 0.05) -> str | None:
    readable, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not readable:
        return None
    return sys.stdin.read(1)


def main(args=None) -> None:
    if not sys.stdin.isatty():
        raise SystemExit(
            "keyboard_teleop.py requires an interactive TTY; run it with ros2 run "
            "from a dedicated terminal instead of embedding it in a launch file"
        )

    rclpy.init(args=args)
    node = KeyboardTeleop()
    old_settings = termios.tcgetattr(sys.stdin)
    keymap = {
        "w": (1.0, 0.0),
        "s": (-1.0, 0.0),
        "a": (0.0, 1.0),
        "d": (0.0, -1.0),
        "q": (1.0, 1.0),
        "e": (1.0, -1.0),
        "z": (-1.0, -1.0),
        "c": (-1.0, 1.0),
    }

    print(HELP)
    print(
        f"\npublishing: {node.command_topic} | "
        f"linear={node.linear_speed:.2f} m/s | angular={node.angular_speed:.2f} rad/s | "
        f"deadman={node.deadman_timeout_s:.2f} s"
    )

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = _read_key(0.05)
            if key is not None:
                key = key.lower()
                if key == "\x03":
                    break
                if key in keymap:
                    node.set_motion(*keymap[key])
                elif key in (" ", "x"):
                    node.stop()
                elif key in ("+", "="):
                    node.adjust_linear(node.linear_step)
                elif key in ("-", "_"):
                    node.adjust_linear(-node.linear_step)
                elif key == "]":
                    node.adjust_angular(node.angular_step)
                elif key == "[":
                    node.adjust_angular(-node.angular_step)
                elif key == "h":
                    print("\n" + HELP)
            node.watchdog()
            node.publish_command()
    finally:
        try:
            node.stop()
            # Send several zero commands so the bridge / simulator receives a stop
            # even when the process exits immediately after a key press.
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=0.02)
                node.publisher.publish(Twist())
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
