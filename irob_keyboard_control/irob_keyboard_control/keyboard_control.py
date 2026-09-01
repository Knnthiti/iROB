#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node

from irob_ros.msg import IROBCommand


REG_NULL = 0
REG_ESTOP = 170
INT16_MIN = -32768
INT16_MAX = 32767

HELP_TEXT = """
iROB keyboard control
---------------------
W/S : forward / backward
A/D : strafe left / right
Q/E : rotate left / right
Space or X : stop
Z : emergency stop
+ or = : increase speed
- or _ : decrease speed
Ctrl+C : quit
"""


class IROBKeyboardControl(Node):
    def __init__(self):
        super().__init__("irob_keyboard_control")

        self.publisher = self.create_publisher(IROBCommand, "iROB_command", 10)

        self.declare_parameter("speed_rpm", 100)
        self.declare_parameter("turn_rpm", 100)
        self.declare_parameter("speed_step_rpm", 25)
        self.declare_parameter("min_speed_rpm", 25)
        self.declare_parameter("max_speed_rpm", 300)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.speed_rpm = self._read_int_parameter("speed_rpm")
        self.turn_rpm = self._read_int_parameter("turn_rpm")
        self.speed_step_rpm = self._read_int_parameter("speed_step_rpm")
        self.min_speed_rpm = self._read_int_parameter("min_speed_rpm")
        self.max_speed_rpm = self._read_int_parameter("max_speed_rpm")
        self.publish_rate_hz = self._read_float_parameter("publish_rate_hz")

        self.current_command = IROBCommand()
        self.command_name = "stop"
        self.set_stop()

    def _read_int_parameter(self, name):
        return int(self.get_parameter(name).value)

    def _read_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        return max(value, 1.0)

    def clamp_rpm(self, value):
        value = int(value)
        value = max(value, INT16_MIN)
        value = min(value, INT16_MAX)
        return value

    def set_motor_command(self, name, lf, lb, rb, rf, reg=REG_NULL):
        msg = IROBCommand()
        msg.reg = int(reg)
        msg.ctk = 0
        msg.motor1_ctrl = self.clamp_rpm(lf)
        msg.motor2_ctrl = self.clamp_rpm(lb)
        msg.motor3_ctrl = self.clamp_rpm(rb)
        msg.motor4_ctrl = self.clamp_rpm(rf)
        msg.cmd_data_pc = 0

        self.current_command = msg
        self.command_name = name

    def set_stop(self):
        self.set_motor_command("stop", 0, 0, 0, 0)

    def set_estop(self):
        self.set_motor_command("emergency stop", 0, 0, 0, 0, REG_ESTOP)

    def apply_key(self, key):
        key = key.lower()
        speed = self.speed_rpm
        turn = self.turn_rpm

        if key == "w":
            self.set_motor_command("forward", speed, speed, -speed, -speed)
        elif key == "s":
            self.set_motor_command("backward", -speed, -speed, speed, speed)
        elif key == "a":
            self.set_motor_command("strafe left", -speed, speed, speed, -speed)
        elif key == "d":
            self.set_motor_command("strafe right", speed, -speed, -speed, speed)
        elif key == "q":
            self.set_motor_command("rotate left", -turn, -turn, -turn, -turn)
        elif key == "e":
            self.set_motor_command("rotate right", turn, turn, turn, turn)
        elif key in (" ", "x"):
            self.set_stop()
        elif key == "z":
            self.set_estop()
        elif key in ("+", "="):
            self.speed_rpm = min(self.max_speed_rpm, self.speed_rpm + self.speed_step_rpm)
            self.turn_rpm = min(self.max_speed_rpm, self.turn_rpm + self.speed_step_rpm)
        elif key in ("-", "_"):
            self.speed_rpm = max(self.min_speed_rpm, self.speed_rpm - self.speed_step_rpm)
            self.turn_rpm = max(self.min_speed_rpm, self.turn_rpm - self.speed_step_rpm)
        else:
            return False

        return True

    def publish_command(self):
        self.publisher.publish(self.current_command)

    def status_line(self):
        msg = self.current_command
        return (
            f"{self.command_name}: "
            f"LF={msg.motor1_ctrl} LB={msg.motor2_ctrl} "
            f"RB={msg.motor3_ctrl} RF={msg.motor4_ctrl} "
            f"speed={self.speed_rpm} turn={self.turn_rpm} reg={msg.reg}"
        )


def open_keyboard_stream():
    try:
        return open("/dev/tty", "r", encoding="utf-8")
    except OSError:
        return sys.stdin


def read_key(input_stream, timeout_sec):
    ready, _, _ = select.select([input_stream], [], [], timeout_sec)
    if not ready:
        return None

    return input_stream.read(1)


def main(args=None):
    rclpy.init(args=args)
    node = IROBKeyboardControl()
    input_stream = open_keyboard_stream()

    if not input_stream.isatty():
        node.get_logger().error("Keyboard control needs an interactive terminal.")
        if input_stream is not sys.stdin:
            input_stream.close()
        node.destroy_node()
        rclpy.shutdown()
        return 1

    terminal_settings = termios.tcgetattr(input_stream)
    publish_period = 1.0 / node.publish_rate_hz

    print(HELP_TEXT)
    print(node.status_line())

    try:
        tty.setcbreak(input_stream.fileno())

        while rclpy.ok():
            key = read_key(input_stream, publish_period)

            if key == "\x03":
                raise KeyboardInterrupt

            if key is not None and node.apply_key(key):
                print(node.status_line())

            node.publish_command()
            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.set_stop()
        for _ in range(5):
            node.publish_command()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)

        termios.tcsetattr(input_stream, termios.TCSADRAIN, terminal_settings)
        if input_stream is not sys.stdin:
            input_stream.close()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
