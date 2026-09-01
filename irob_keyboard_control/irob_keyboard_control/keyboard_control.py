#!/usr/bin/env python3
"""node ควบคุม iROB ด้วย keyboard ผ่าน ROS2 topic /iROB_command."""

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node

from irob_ros.msg import IROBCommand


# ค่า register ที่ส่งไปให้ ESP32
REG_NULL = 0
REG_ESTOP = 170

# ช่วงค่าที่ message int16 รองรับ
INT16_MIN = -32768
INT16_MAX = 32767

HELP_TEXT = """
ควบคุม iROB ด้วย keyboard
-------------------------
W/S : เดินหน้า / ถอยหลัง
A/D : สไลด์ซ้าย / สไลด์ขวา
Q/E : หมุนซ้าย / หมุนขวา
Space หรือ X : หยุด
Z : emergency stop
+ หรือ = : เพิ่มความเร็ว
- หรือ _ : ลดความเร็ว
Ctrl+C : ออกจากโปรแกรม และส่ง stop
"""


class IROBKeyboardControl(Node):
    """node ROS2 ที่อ่านปุ่ม keyboard แล้ว publish IROBCommand."""

    def __init__(self):
        # ตั้งชื่อ node ภายใน rclpy
        super().__init__("irob_keyboard_control")

        # publisher สำหรับส่งคำสั่ง motor ไปยัง topic /iROB_command
        # node iROB_CMD จะ subscribe topic นี้แล้วส่งต่อไป ESP32
        self.publisher = self.create_publisher(IROBCommand, "iROB_command", 10)

        # parameter เหล่านี้ปรับได้จาก launch file หรือ ros2 param
        self.declare_parameter("speed_rpm", 100)
        self.declare_parameter("turn_rpm", 100)
        self.declare_parameter("speed_step_rpm", 25)
        self.declare_parameter("min_speed_rpm", 25)
        self.declare_parameter("max_speed_rpm", 300)
        self.declare_parameter("publish_rate_hz", 20.0)

        # อ่าน parameter แล้วแปลง type ให้พร้อมใช้งาน
        self.speed_rpm = self._read_int_parameter("speed_rpm")
        self.turn_rpm = self._read_int_parameter("turn_rpm")
        self.speed_step_rpm = self._read_int_parameter("speed_step_rpm")
        self.min_speed_rpm = self._read_int_parameter("min_speed_rpm")
        self.max_speed_rpm = self._read_int_parameter("max_speed_rpm")
        self.publish_rate_hz = self._read_float_parameter("publish_rate_hz")

        # เก็บคำสั่งล่าสุดไว้ เพื่อ publish ซ้ำต่อเนื่องตาม publish_rate_hz
        # ถ้าไม่ publish ต่อเนื่อง ESP32 จะ timeout และหยุด motor
        self.current_command = IROBCommand()
        self.command_name = "หยุด"
        self.set_stop()

    def _read_int_parameter(self, name):
        """อ่าน ROS2 parameter แล้วแปลงเป็น int."""
        return int(self.get_parameter(name).value)

    def _read_float_parameter(self, name):
        """อ่าน ROS2 parameter แล้วแปลงเป็น float โดยกัน publish rate ต่ำเกินไป."""
        value = float(self.get_parameter(name).value)
        return max(value, 1.0)

    def clamp_rpm(self, value):
        """จำกัดค่า RPM ให้อยู่ในช่วง int16 ก่อนใส่ message."""
        value = int(value)
        value = max(value, INT16_MIN)
        value = min(value, INT16_MAX)
        return value

    def set_motor_command(self, name, lf, lb, rb, rf, reg=REG_NULL):
        """สร้าง message คำสั่ง motor ตามลำดับล้อ LF, LB, RB, RF."""
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
        """ตั้งคำสั่งหยุดแบบปกติ."""
        self.set_motor_command("หยุด", 0, 0, 0, 0)

    def set_estop(self):
        """ตั้งคำสั่ง emergency stop โดยส่ง reg พิเศษไปยัง ESP32."""
        self.set_motor_command("emergency stop", 0, 0, 0, 0, REG_ESTOP)

    def apply_key(self, key):
        """แปลงปุ่ม keyboard เป็นคำสั่ง motor."""
        key = key.lower()
        speed = self.speed_rpm
        turn = self.turn_rpm

        # การ map ล้อ mecanum:
        # motor1 = LF, motor2 = LB, motor3 = RB, motor4 = RF
        if key == "w":
            self.set_motor_command("เดินหน้า", speed, speed, -speed, -speed)
        elif key == "s":
            self.set_motor_command("ถอยหลัง", -speed, -speed, speed, speed)
        elif key == "a":
            self.set_motor_command("สไลด์ซ้าย", -speed, speed, speed, -speed)
        elif key == "d":
            self.set_motor_command("สไลด์ขวา", speed, -speed, -speed, speed)
        elif key == "q":
            self.set_motor_command("หมุนซ้าย", -turn, -turn, -turn, -turn)
        elif key == "e":
            self.set_motor_command("หมุนขวา", turn, turn, turn, turn)
        elif key in (" ", "x"):
            self.set_stop()
        elif key == "z":
            self.set_estop()
        elif key in ("+", "="):
            # เพิ่มความเร็ว translation และ rotation พร้อมกัน
            self.speed_rpm = min(self.max_speed_rpm, self.speed_rpm + self.speed_step_rpm)
            self.turn_rpm = min(self.max_speed_rpm, self.turn_rpm + self.speed_step_rpm)
        elif key in ("-", "_"):
            # ลดความเร็ว แต่ไม่ต่ำกว่า min_speed_rpm
            self.speed_rpm = max(self.min_speed_rpm, self.speed_rpm - self.speed_step_rpm)
            self.turn_rpm = max(self.min_speed_rpm, self.turn_rpm - self.speed_step_rpm)
        else:
            # ปุ่มอื่นไม่เปลี่ยนคำสั่ง
            return False

        return True

    def publish_command(self):
        """publish คำสั่งล่าสุดไปยัง /iROB_command."""
        self.publisher.publish(self.current_command)

    def status_line(self):
        """สร้างข้อความสถานะสำหรับแสดงใน terminal."""
        msg = self.current_command
        return (
            f"{self.command_name}: "
            f"LF={msg.motor1_ctrl} LB={msg.motor2_ctrl} "
            f"RB={msg.motor3_ctrl} RF={msg.motor4_ctrl} "
            f"speed={self.speed_rpm} turn={self.turn_rpm} reg={msg.reg}"
        )


def open_keyboard_stream():
    """เปิด terminal จริงเพื่อรับปุ่ม แม้ node จะถูกเปิดผ่าน ros2 launch."""
    try:
        return open("/dev/tty", "r", encoding="utf-8")
    except OSError:
        return sys.stdin


def read_key(input_stream, timeout_sec):
    """อ่านปุ่ม 1 ตัวแบบไม่ block เกิน timeout_sec."""
    ready, _, _ = select.select([input_stream], [], [], timeout_sec)
    if not ready:
        return None

    return input_stream.read(1)


def main(args=None):
    """จุดเริ่มทำงานของ console script irob_keyboard."""
    rclpy.init(args=args)
    node = IROBKeyboardControl()
    input_stream = open_keyboard_stream()

    # ถ้าไม่ได้รันใน terminal interactive จะรับปุ่มไม่ได้
    if not input_stream.isatty():
        node.get_logger().error("Keyboard control ต้องรันใน terminal ที่รับปุ่มได้")
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
        # ตั้ง terminal เป็น cbreak เพื่ออ่านปุ่มทีละตัวโดยไม่ต้องกด Enter
        tty.setcbreak(input_stream.fileno())

        while rclpy.ok():
            # อ่านปุ่มตามคาบ publish เพื่อให้ส่งคำสั่งซ้ำได้ต่อเนื่อง
            key = read_key(input_stream, publish_period)

            if key == "\x03":
                raise KeyboardInterrupt

            if key is not None and node.apply_key(key):
                print(node.status_line())

            # publish คำสั่งล่าสุดทุก loop เพื่อกัน ESP32 command timeout
            node.publish_command()
            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        # ตอนออกจากโปรแกรม ส่ง stop หลายครั้งเพื่อให้ ESP32 หยุดแน่นอน
        node.set_stop()
        for _ in range(5):
            node.publish_command()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)

        # คืนค่า terminal และปิด ROS2 node
        termios.tcsetattr(input_stream, termios.TCSADRAIN, terminal_settings)
        if input_stream is not sys.stdin:
            input_stream.close()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
