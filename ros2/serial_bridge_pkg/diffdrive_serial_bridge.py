#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

import serial
from serial import SerialException

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformBroadcaster


class DiffDriveSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("diffdrive_serial_bridge")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.declare_parameter("wheel_radius", 0.0325)   # 65 mm diameter / 2
        self.declare_parameter("wheel_base", 0.18)       # MEDIR en el robot
        self.declare_parameter("ticks_per_rev", 1000.0)  # AJUSTAR luego

        self.declare_parameter("max_wheel_linear_speed", 0.60)
        self.declare_parameter("min_pwm", 70)
        self.declare_parameter("max_pwm", 255)

        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("publish_tf", True)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)

        self.max_wheel_linear_speed = float(
            self.get_parameter("max_wheel_linear_speed").value
        )
        self.min_pwm = int(self.get_parameter("min_pwm").value)
        self.max_pwm = int(self.get_parameter("max_pwm").value)

        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.ticks_pub = self.create_publisher(Int64MultiArray, "/wheel_ticks", 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        # -----------------------------
        # Internal state
        # -----------------------------
        self.serial_conn = None

        self.latest_cmd_v = 0.0
        self.latest_cmd_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.last_sent_left_pwm = 0
        self.last_sent_right_pwm = 0

        self.left_ticks = 0
        self.right_ticks = 0

        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.prev_enc_time = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.open_serial()

        # timers
        self.read_timer = self.create_timer(0.01, self.read_serial_timer_cb)   # 100 Hz
        self.cmd_timer = self.create_timer(0.05, self.send_command_timer_cb)   # 20 Hz

        self.get_logger().info("diffdrive_serial_bridge listo.")

    # -----------------------------
    # Serial
    # -----------------------------
    def open_serial(self) -> None:
        try:
            self.serial_conn = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.05,
                write_timeout=0.05,
            )

            # El ESP32 suele reiniciarse al abrir el puerto
            time.sleep(2.0)

            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()

            self.get_logger().info(
                f"Puerto serial abierto: {self.port} @ {self.baudrate}"
            )

            self.send_line("STREAM,1")
            self.send_line("CFG?")
            self.send_line("E?")
        except SerialException as e:
            self.serial_conn = None
            self.get_logger().error(f"No se pudo abrir {self.port}: {e}")

    def send_line(self, line: str) -> None:
        if self.serial_conn is None:
            return

        try:
            self.serial_conn.write((line + "\n").encode("utf-8"))
        except SerialException as e:
            self.get_logger().error(f"Error escribiendo serial: {e}")
            self.serial_conn = None

    # -----------------------------
    # cmd_vel -> PWM
    # -----------------------------
    def cmd_vel_callback(self, msg: Twist) -> None:
        self.latest_cmd_v = float(msg.linear.x)
        self.latest_cmd_w = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def wheel_speed_to_pwm(self, wheel_linear_speed: float) -> int:
        if abs(wheel_linear_speed) < 1e-6:
            return 0

        sign = 1 if wheel_linear_speed > 0.0 else -1
        speed_mag = abs(wheel_linear_speed)

        norm = min(speed_mag / self.max_wheel_linear_speed, 1.0)
        pwm = int(self.min_pwm + norm * (self.max_pwm - self.min_pwm))
        pwm = max(self.min_pwm, min(self.max_pwm, pwm))

        return sign * pwm

    def send_command_timer_cb(self) -> None:
        if self.serial_conn is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9

        if dt > self.command_timeout:
            left_pwm = 0
            right_pwm = 0
        else:
            v_left = self.latest_cmd_v - (self.latest_cmd_w * self.wheel_base / 2.0)
            v_right = self.latest_cmd_v + (self.latest_cmd_w * self.wheel_base / 2.0)

            left_pwm = self.wheel_speed_to_pwm(v_left)
            right_pwm = self.wheel_speed_to_pwm(v_right)

        self.send_line(f"M,{left_pwm},{right_pwm}")
        self.last_sent_left_pwm = left_pwm
        self.last_sent_right_pwm = right_pwm

    # -----------------------------
    # Read serial / parse ENC
    # -----------------------------
    def read_serial_timer_cb(self) -> None:
        if self.serial_conn is None:
            return

        try:
            while self.serial_conn.in_waiting > 0:
                raw = self.serial_conn.readline()
                if not raw:
                    break

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                self.get_logger().info(f"SERIAL RX: {line}")
                self.handle_serial_line(line)

        except SerialException as e:
            self.get_logger().error(f"Error leyendo serial: {e}")
            self.serial_conn = None

    def handle_serial_line(self, line: str) -> None:
        if line.startswith("ENC,"):
            parts = line.split(",")
            if len(parts) != 3:
                return

            try:
                left = int(parts[1])
                right = int(parts[2])
            except ValueError:
                return

            self.left_ticks = left
            self.right_ticks = right

            self.publish_ticks()
            self.update_and_publish_odom()
            return

        if line.startswith("ERR,"):
            self.get_logger().warn(line)
            return

        if line.startswith("OK,"):
            self.get_logger().info(line)
            return

        self.get_logger().info(f"Serial: {line}")

    # -----------------------------
    # Publish raw ticks
    # -----------------------------
    def publish_ticks(self) -> None:
        msg = Int64MultiArray()
        msg.data = [self.left_ticks, self.right_ticks]
        self.ticks_pub.publish(msg)

    # -----------------------------
    # Odometry
    # -----------------------------
    def update_and_publish_odom(self) -> None:
        now = self.get_clock().now()

        if self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
            self.prev_enc_time = now
            return

        dt = (now - self.prev_enc_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        delta_left_ticks = self.left_ticks - self.prev_left_ticks
        delta_right_ticks = self.right_ticks - self.prev_right_ticks

        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        d_left = delta_left_ticks * meters_per_tick
        d_right = delta_right_ticks * meters_per_tick

        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.wheel_base

        theta_mid = self.theta + 0.5 * d_theta
        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta += d_theta

        vx = d_center / dt
        vth = d_theta / dt

        self.publish_odom(now, vx, vth)

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.prev_enc_time = now

    def publish_odom(self, stamp, vx: float, vth: float) -> None:
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp.to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame

            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf_msg)

    # -----------------------------
    # Shutdown
    # -----------------------------
    def destroy_node(self):
        try:
            self.send_line("S")
        except Exception:
            pass

        try:
            if self.serial_conn is not None:
                self.serial_conn.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
