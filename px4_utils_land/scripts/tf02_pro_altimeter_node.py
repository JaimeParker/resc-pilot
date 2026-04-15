#!/usr/bin/env python3
"""
TF02-Pro UART to ROS LaserScan bridge.

Publishes a single-beam sensor_msgs/LaserScan message on /scan by default,
which can be consumed directly by PX4CtrlFSM.
"""

from __future__ import annotations

import math
import threading
import time

import rospy
from sensor_msgs.msg import LaserScan

try:
    import serial
    from serial import SerialException
except ImportError as exc:
    raise ImportError("pyserial is required. Install python3-serial or pip install pyserial") from exc


FRAME_HEADER = b"\x59\x59"
FRAME_SIZE = 9


def parse_frame(frame: bytes) -> tuple[int, int]:
    """Return (distance_cm, strength) for one valid TF02-Pro frame."""
    if len(frame) != FRAME_SIZE:
        raise ValueError("Unexpected frame size")
    if frame[:2] != FRAME_HEADER:
        raise ValueError("Invalid frame header")

    checksum = sum(frame[:8]) & 0xFF
    if checksum != frame[8]:
        raise ValueError("Checksum mismatch")

    distance_cm = frame[2] | (frame[3] << 8)
    strength = frame[4] | (frame[5] << 8)
    return distance_cm, strength


class Tf02ProAltimeterNode:
    def __init__(self) -> None:
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baudrate = int(rospy.get_param("~baudrate", 115200))
        self.serial_timeout = float(rospy.get_param("~serial_timeout", 0.1))
        self.reconnect_delay = float(rospy.get_param("~reconnect_delay", 1.0))

        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.range_min = float(rospy.get_param("~range_min", 0.05))
        self.range_max = float(rospy.get_param("~range_max", 30.0))
        self.signal_threshold = int(rospy.get_param("~signal_threshold", 0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))

        self._pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=10)
        self._lock = threading.Lock()
        self._latest_range_m = math.nan
        self._latest_strength = 0
        self._latest_stamp = rospy.Time(0)

        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        rospy.loginfo("[TF02] Node started. port=%s baudrate=%d publish_topic=%s",
                      self.port, self.baudrate, self.scan_topic)

    def _read_frame(self, port: serial.Serial) -> tuple[int, int]:
        while not rospy.is_shutdown():
            first = port.read(1)
            if not first:
                raise TimeoutError("Timeout waiting first header byte")
            if first != FRAME_HEADER[:1]:
                continue

            second = port.read(1)
            if not second:
                raise TimeoutError("Timeout waiting second header byte")
            if second != FRAME_HEADER[1:2]:
                continue

            payload = port.read(FRAME_SIZE - 2)
            if len(payload) != FRAME_SIZE - 2:
                raise TimeoutError("Timeout waiting frame payload")

            return parse_frame(FRAME_HEADER + payload)

        raise RuntimeError("ROS shutdown")

    def _read_loop(self) -> None:
        while not rospy.is_shutdown():
            try:
                with serial.Serial(self.port, self.baudrate, timeout=self.serial_timeout) as ser:
                    ser.reset_input_buffer()
                    rospy.loginfo("[TF02] Serial connected: %s @ %d", self.port, self.baudrate)

                    while not rospy.is_shutdown():
                        try:
                            distance_cm, strength = self._read_frame(ser)
                        except TimeoutError:
                            continue
                        except ValueError:
                            continue

                        range_m = float(distance_cm) / 100.0

                        if strength < self.signal_threshold:
                            range_m = math.nan
                        elif range_m < self.range_min or range_m > self.range_max:
                            range_m = math.nan

                        with self._lock:
                            self._latest_range_m = range_m
                            self._latest_strength = strength
                            self._latest_stamp = rospy.Time.now()
            except SerialException as exc:
                rospy.logwarn_throttle(2.0, "[TF02] Serial open/read failed on %s: %s", self.port, str(exc))
                time.sleep(self.reconnect_delay)

    def _build_scan(self, stamp: rospy.Time, value: float) -> LaserScan:
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.angle_min = 0.0
        msg.angle_max = 0.0
        msg.angle_increment = 0.0
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / max(self.publish_rate, 1e-6)
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = [value]
        msg.intensities = [float(self._latest_strength)]
        return msg

    def spin(self) -> None:
        rate = rospy.Rate(max(self.publish_rate, 1.0))
        while not rospy.is_shutdown():
            with self._lock:
                value = self._latest_range_m
                stamp = self._latest_stamp if self._latest_stamp != rospy.Time(0) else rospy.Time.now()

            self._pub.publish(self._build_scan(stamp, value))
            rate.sleep()


def main() -> None:
    rospy.init_node("tf02_pro_altimeter_node")
    node = Tf02ProAltimeterNode()
    node.spin()


if __name__ == "__main__":
    main()
