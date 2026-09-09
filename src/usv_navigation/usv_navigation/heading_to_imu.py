#!/usr/bin/env python3
"""Combine dual-antenna RTK yaw with the LiDAR IMU roll/pitch."""

import math
import time
from typing import Optional

from geometry_msgs.msg import Quaternion
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def _rpy_from_quaternion(q: Quaternion):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = (math.copysign(math.pi / 2.0, sinp)
             if abs(sinp) >= 1.0 else math.asin(sinp))
    return roll, pitch


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class HeadingToImu(Node):
    """Publish an IMU whose yaw is supplied by RTK and tilt by LiDAR IMU."""

    def __init__(self):
        super().__init__('heading_to_imu')
        self.declare_parameter('imu_topic', '/unilidar/imu')
        self.declare_parameter('heading_topic', '/gps/heading')
        self.declare_parameter('output_topic', '/gps/heading_imu')
        self.declare_parameter('yaw_variance', 0.01)
        self.declare_parameter('require_heading', True)
        self.declare_parameter('heading_timeout_s', 2.5)

        self._yaw: Optional[float] = None
        self._heading_time = 0.0
        self._yaw_variance = float(self.get_parameter('yaw_variance').value)
        self._require_heading = bool(self.get_parameter('require_heading').value)
        self._heading_timeout = float(
            self.get_parameter('heading_timeout_s').value)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(
            Imu, str(self.get_parameter('output_topic').value), qos)
        self.create_subscription(
            Float64, str(self.get_parameter('heading_topic').value),
            self._on_heading, qos)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value), self._on_imu, qos)
        self.get_logger().info(
            'combining RTK ENU yaw with LiDAR IMU roll/pitch on '
            f"{self.get_parameter('output_topic').value}")

    def _on_heading(self, msg: Float64):
        if math.isfinite(msg.data):
            self._yaw = float(msg.data)
            self._heading_time = time.monotonic()

    def _on_imu(self, msg: Imu):
        heading_fresh = (self._yaw is not None and
                         time.monotonic() - self._heading_time <=
                         self._heading_timeout)
        if self._require_heading and not heading_fresh:
            return
        out = Imu()
        out.header = msg.header
        roll, pitch = _rpy_from_quaternion(msg.orientation)
        out.orientation = _quaternion_from_rpy(
            roll, pitch, self._yaw if self._yaw is not None else 0.0)
        out.orientation_covariance = list(msg.orientation_covariance)
        # All-zero covariance means "unknown" in sensor_msgs/Imu.  Leaving
        # roll/pitch at zero would incorrectly tell robot_localization that
        # those axes are perfect measurements.
        if (not all(math.isfinite(value)
                    for value in out.orientation_covariance[:3]) or
                all(value == 0.0 for value in out.orientation_covariance[:3]) or
                out.orientation_covariance[0] < 0.0):
            out.orientation_covariance[0] = 0.05
            out.orientation_covariance[4] = 0.05
            out.orientation_covariance[8] = self._yaw_variance
        else:
            out.orientation_covariance[8] = self._yaw_variance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingToImu()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
