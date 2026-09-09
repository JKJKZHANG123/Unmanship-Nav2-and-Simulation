#!/usr/bin/env python3
"""Republish Nav2's metric plan as a WGS84 ``geographic_msgs/GeoPath``."""

import json
import math
from typing import Optional

from geographic_msgs.msg import GeoPath, GeoPoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, NavSatStatus
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener

from .geodesy import (lat_lon_to_utm, quaternion_to_yaw, utm_to_lat_lon,
                      validate_lat_lon)


class GeodeticPathPublisher(Node):
    def __init__(self):
        super().__init__('geodetic_path_publisher')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('geo_path_topic', '/plan_geodetic')
        self.declare_parameter('json_topic', '/plan_geodetic_json')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('tf_timeout_s', 0.5)
        self.declare_parameter('altitude_fallback_m', 0.0)
        self.declare_parameter('min_point_spacing_m', 0.0)
        self.declare_parameter('target_topic', '/target/fix')
        self.declare_parameter('output_port', '')
        self.declare_parameter('output_baud', 115200)

        self._boat: Optional[NavSatFix] = None
        self._last_path_signature = None
        self._last_serial_signature = None
        self._last_plan: Optional[Path] = None
        self._utm_frame = str(self.get_parameter('utm_frame').value)
        self._tf_timeout = float(self.get_parameter('tf_timeout_s').value)
        self._alt_fallback = float(
            self.get_parameter('altitude_fallback_m').value)
        self._min_spacing = max(0.0, float(
            self.get_parameter('min_point_spacing_m').value))
        self._target: Optional[NavSatFix] = None
        self._output_port = str(self.get_parameter('output_port').value)
        self._output_baud = int(self.get_parameter('output_baud').value)
        self._output_serial = None
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
        plan_topic = str(self.get_parameter('plan_topic').value)
        self._geo_path_topic = str(self.get_parameter('geo_path_topic').value)
        self._json_topic = str(self.get_parameter('json_topic').value)
        self._geo_pub = self.create_publisher(GeoPath, self._geo_path_topic, q)
        self._json_pub = self.create_publisher(String, self._json_topic, q)
        self.create_subscription(Path, plan_topic, self._on_plan, q)
        self.create_subscription(NavSatFix,
                                 str(self.get_parameter('gps_topic').value),
                                 self._on_boat, q)
        self.create_subscription(NavSatFix,
                                 str(self.get_parameter('target_topic').value),
                                 self._on_target, q)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.create_timer(1.0, self._retry_latest_plan)
        self.get_logger().info(
            f'{plan_topic} -> {self._geo_path_topic}, {self._json_topic}')

    def _on_boat(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX and validate_lat_lon(
                msg.latitude, msg.longitude):
            self._boat = msg

    def _on_target(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX and validate_lat_lon(
                msg.latitude, msg.longitude):
            self._target = msg

    def _on_plan(self, msg: Path):
        self._last_plan = msg
        self._publish_plan(msg)

    def _retry_latest_plan(self):
        # Only retry a non-empty plan: the retry timer exists to re-geocode a
        # path once TF/GPS become available.  Re-publishing an empty (cleared)
        # path every second would flood downstream consumers with clear frames.
        if self._last_plan is not None and self._last_plan.poses:
            self._publish_plan(self._last_plan)

    def _write_serial(self, payload: str):
        if not self._output_port:
            return
        if self._output_serial is None:
            try:
                import serial
                self._output_serial = serial.Serial(
                    self._output_port, self._output_baud, timeout=0.1)
                self.get_logger().info(
                    f'opened geodetic path output {self._output_port}')
            except Exception as exc:
                self.get_logger().warn(
                    f'cannot open path output {self._output_port}: {exc}')
                return
        try:
            self._output_serial.write((payload + '\n').encode('utf-8'))
            self._output_serial.flush()
        except Exception as exc:
            self.get_logger().warn(f'path serial write failed: {exc}')
            try:
                self._output_serial.close()
            except Exception:
                pass
            self._output_serial = None

    def _publish_plan(self, msg: Path):
        if not msg.poses:
            # A cleared /plan (cancel / abort) must propagate as an empty
            # geodetic path, otherwise a downstream FC mission or serial sink
            # keeps a stale route.  An empty GeoPath with no poses is the
            # explicit "clear" signal; do not silently drop it.
            empty = GeoPath()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = 'wgs84'
            self._geo_pub.publish(empty)
            self._json_pub.publish(String(data='{"frame":"wgs84","points":[]}'))
            self.get_logger().info('published empty /plan_geodetic (route cleared)')
            return
        if self._boat is None:
            self.get_logger().warn('cannot geocode /plan: no valid /gps/fix')
            return
        try:
            _, _, zone, northern = lat_lon_to_utm(
                self._boat.latitude, self._boat.longitude)
            transform = self._tf.lookup_transform(
                self._utm_frame, msg.header.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout))
            geo = GeoPath()
            geo.header.stamp = self.get_clock().now().to_msg()
            geo.header.frame_id = 'wgs84'

            # Transform all valid poses first, then decimate while preserving
            # both endpoints. Losing the final pose here would make the FC
            # mission stop short of the requested target.
            samples = []
            for index, pose in enumerate(msg.poses):
                local_x = float(pose.pose.position.x)
                local_y = float(pose.pose.position.y)
                if not math.isfinite(local_x) or not math.isfinite(local_y):
                    continue
                utm_pose = do_transform_pose_stamped(pose, transform)
                east = float(utm_pose.pose.position.x)
                north = float(utm_pose.pose.position.y)
                if not math.isfinite(east) or not math.isfinite(north):
                    continue
                lat, lon = utm_to_lat_lon(east, north, zone, northern)
                samples.append({
                    'index': index,
                    'latitude': lat,
                    'longitude': lon,
                    'altitude': (float(self._boat.altitude)
                                 if math.isfinite(self._boat.altitude)
                                 else self._alt_fallback),
                    'heading_rad_enu': quaternion_to_yaw(
                        utm_pose.pose.orientation),
                    'local_x': local_x,
                    'local_y': local_y,
                    'utm_easting': east,
                    'utm_northing': north,
                    '_orientation': utm_pose.pose.orientation,
                })
            if not samples:
                return

            if self._min_spacing > 0.0 and len(samples) > 2:
                records = [samples[0]]
                for sample in samples[1:-1]:
                    distance = math.hypot(
                        sample['utm_easting'] - records[-1]['utm_easting'],
                        sample['utm_northing'] - records[-1]['utm_northing'])
                    if distance >= self._min_spacing:
                        records.append(sample)
                final = samples[-1]
                final_distance = math.hypot(
                    final['utm_easting'] - records[-1]['utm_easting'],
                    final['utm_northing'] - records[-1]['utm_northing'])
                if final_distance > 0.001:
                    records.append(final)
            else:
                records = samples

            for record in records:
                geo_pose = GeoPoseStamped()
                geo_pose.header.stamp = geo.header.stamp
                geo_pose.header.frame_id = 'wgs84'
                geo_pose.pose.position.latitude = record['latitude']
                geo_pose.pose.position.longitude = record['longitude']
                geo_pose.pose.position.altitude = record['altitude']
                geo_pose.pose.orientation = record['_orientation']
                geo.poses.append(geo_pose)
                del record['_orientation']

            signature = tuple(
                (round(record['latitude'], 7), round(record['longitude'], 7),
                 round(record['heading_rad_enu'], 4),
                 round(record['altitude'], 2))
                for record in records)
            self._geo_pub.publish(geo)
            payload = {
                'frame': 'wgs84',
                'source_frame': msg.header.frame_id,
                'stamp': {'sec': geo.header.stamp.sec,
                          'nanosec': geo.header.stamp.nanosec},
                'target': ({
                    'latitude': self._target.latitude,
                    'longitude': self._target.longitude,
                    'altitude': self._target.altitude,
                } if self._target is not None else None),
                'points': records,
            }
            json_text = json.dumps(payload, separators=(',', ':'))
            self._json_pub.publish(String(data=json_text))
            # ROS path topics are intentionally refreshed while TF/GPS settle,
            # but a serial consumer gets one frame per actual path change.
            if signature != self._last_serial_signature:
                self._write_serial(json_text)
                self._last_serial_signature = signature
            if signature != self._last_path_signature:
                self._last_path_signature = signature
                self.get_logger().info(
                    f'published {len(records)} geodetic path points on '
                    f'{self._geo_path_topic}')
        except (TransformException, ValueError) as exc:
            self.get_logger().warn(f'cannot geocode /plan: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = GeodeticPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._output_serial is not None:
            node._output_serial.close()
        node.destroy_node()
        rclpy.shutdown()
