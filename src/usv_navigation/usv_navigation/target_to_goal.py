#!/usr/bin/env python3
"""Convert a target NavSatFix into a metric Nav2 ``/goal_pose``."""

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from std_msgs.msg import String

from .geodesy import (lat_lon_to_utm, validate_lat_lon, yaw_to_quaternion)


class TargetToGoal(Node):
    def __init__(self):
        super().__init__('target_to_goal')
        self.declare_parameter('target_topic', '/target/fix')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('global_frame', 'camera_init')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('goal_altitude_m', 0.0)
        self.declare_parameter('publish_period_s', 0.2)
        self.declare_parameter('tf_timeout_s', 0.5)
        self.declare_parameter('max_target_distance_m', 5000.0)
        self.declare_parameter('reject_zone_mismatch', True)
        self.declare_parameter('status_topic', '/usv/goal_status')
        self.declare_parameter('retry_period_s', 1.0)
        self.declare_parameter('max_goal_retries', 0)

        self._target: Optional[NavSatFix] = None
        self._boat: Optional[NavSatFix] = None
        self._target_key = None
        self._published_key = None
        self._last_warn_ns = 0
        self._global_frame = str(self.get_parameter('global_frame').value)
        self._utm_frame = str(self.get_parameter('utm_frame').value)
        self._goal_altitude = float(self.get_parameter('goal_altitude_m').value)
        self._tf_timeout = float(self.get_parameter('tf_timeout_s').value)
        self._max_distance = float(
            self.get_parameter('max_target_distance_m').value)
        self._reject_zone_mismatch = bool(
            self.get_parameter('reject_zone_mismatch').value)
        self._retry_period = float(self.get_parameter('retry_period_s').value)
        self._max_retries = int(self.get_parameter('max_goal_retries').value)
        self._retry_at_ns = 0
        self._goal_accepted = False
        self._goal_attempts = 0

        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        target_topic = str(self.get_parameter('target_topic').value)
        gps_topic = str(self.get_parameter('gps_topic').value)
        self._goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('goal_topic').value), q)
        self.create_subscription(NavSatFix, target_topic, self._on_target, q)
        self.create_subscription(NavSatFix, gps_topic, self._on_boat, q)
        self.create_subscription(
            String, str(self.get_parameter('status_topic').value),
            self._on_goal_status, q)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.create_timer(float(self.get_parameter('publish_period_s').value),
                          self._try_publish)
        self.get_logger().info(
            f'{target_topic} -> {self.get_parameter("goal_topic").value}; '
            f'waiting for {self._utm_frame} -> {self._global_frame} TF')

    @staticmethod
    def _valid_fix(msg: NavSatFix) -> bool:
        return (msg.status.status >= NavSatStatus.STATUS_FIX and
                validate_lat_lon(msg.latitude, msg.longitude))

    def _on_boat(self, msg):
        if self._valid_fix(msg):
            self._boat = msg

    def _on_target(self, msg):
        if self._valid_fix(msg):
            new_key = (round(msg.latitude, 8), round(msg.longitude, 8),
                       round(msg.altitude, 2))
            if new_key != self._target_key:
                self._goal_accepted = False
                self._retry_at_ns = 0
                self._goal_attempts = 0
                self._published_key = None
                self._target_key = new_key
            self._target = msg

    def _on_goal_status(self, msg: String):
        text = msg.data
        if 'ACCEPTED' in text or 'EXECUTING' in text:
            self._goal_accepted = True
            return
        # The gateway can reject a goal while Nav2 costmaps are still starting.
        # Retry only that startup condition; a geofence/collision rejection is
        # intentionally left to the operator instead of being spammed.
        retryable = ('no camera_init costmap' in text or
                     'no camera_init pose' in text or
                     'waiting for Nav2 action server' in text)
        if retryable and not self._goal_accepted:
            now = self.get_clock().now().nanoseconds
            self._retry_at_ns = now + int(self._retry_period * 1e9)

    def _warn(self, text):
        now = self.get_clock().now().nanoseconds
        if now - self._last_warn_ns > 5_000_000_000:
            self.get_logger().warn(text)
            self._last_warn_ns = now

    def _try_publish(self):
        if self._target is None or self._boat is None:
            self._warn('waiting for valid /target/fix and /gps/fix')
            return
        target_key = (round(self._target.latitude, 8),
                      round(self._target.longitude, 8),
                      round(self._target.altitude, 2))
        now_ns = self.get_clock().now().nanoseconds
        if target_key == self._published_key:
            if (self._goal_accepted or
                    (self._retry_at_ns == 0 or now_ns < self._retry_at_ns)):
                return
        elif self._retry_at_ns != 0 and now_ns < self._retry_at_ns:
            return
        if (self._max_retries > 0 and
                self._goal_attempts >= self._max_retries):
            self._warn('target goal retry limit reached')
            return
        self._goal_attempts += 1
        try:
            bx, by, boat_zone, boat_north = lat_lon_to_utm(
                self._boat.latitude, self._boat.longitude)
            tx, ty, target_zone, target_north = lat_lon_to_utm(
                self._target.latitude, self._target.longitude)
            if boat_zone != target_zone or boat_north != target_north:
                message = (f'target UTM zone {target_zone} differs from boat '
                           f'zone {boat_zone}; use a nearby target or configure '
                           'a local ENU projection')
                if self._reject_zone_mismatch:
                    self._retry_at_ns = (now_ns +
                                         int(self._retry_period * 1e9))
                    self._warn(message)
                    return
                self.get_logger().warn(message)
            distance = math.hypot(tx - bx, ty - by)
            if self._max_distance > 0.0 and distance > self._max_distance:
                self._retry_at_ns = (now_ns +
                                     int(self._retry_period * 1e9))
                self._warn(f'target distance {distance:.1f}m exceeds '
                           f'{self._max_distance:.1f}m')
                return
            yaw_utm = math.atan2(ty - by, tx - bx) if distance > 0.01 else 0.0
            pose_utm = PoseStamped()
            pose_utm.header.stamp = self.get_clock().now().to_msg()
            pose_utm.header.frame_id = self._utm_frame
            pose_utm.pose.position.x = tx
            pose_utm.pose.position.y = ty
            pose_utm.pose.position.z = self._goal_altitude
            pose_utm.pose.orientation = yaw_to_quaternion(yaw_utm)
            transform = self._tf.lookup_transform(
                self._global_frame, self._utm_frame, rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout))
            goal = do_transform_pose_stamped(pose_utm, transform)
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = self._global_frame
            self._goal_pub.publish(goal)


            key = (round(self._target.latitude, 8),
                   round(self._target.longitude, 8),
                   round(self._target.altitude, 2))
            if key != self._published_key:
                self._published_key = key
                self._retry_at_ns = 0
                self.get_logger().info(
                    f'geodetic target -> /goal_pose: '
                    f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) '
                    f'camera_init, distance={distance:.1f}m')
        except (TransformException, ValueError) as exc:
            self._retry_at_ns = (self.get_clock().now().nanoseconds +
                                 int(self._retry_period * 1e9))
            self._warn(f'cannot transform target to {self._global_frame}: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = TargetToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
