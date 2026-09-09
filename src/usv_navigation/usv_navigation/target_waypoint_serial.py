#!/usr/bin/env python3
"""Read target WGS84 coordinates from a serial port.

Single-waypoint lines::

    TARGET,31.230416,121.473701
    31.230416,121.473701
    {"latitude": 31.230416, "longitude": 121.473701}

Multi-waypoint lines (published on ``/target/geopath`` as a GeoPath)::

    WAYPOINTS,31.2304,121.4737,31.2310,121.4740
    {"waypoints":[{"lat":31.2304,"lon":121.4737},{"lat":31.2310,"lon":121.4740}]}

An optional altitude is accepted as the third value of each point.
"""

import json
import math
import re
from typing import List, Optional, Tuple

from geographic_msgs.msg import GeoPath, GeoPoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .geodesy import validate_lat_lon, yaw_to_quaternion

try:
    import serial
except ImportError:  # pragma: no cover - exercised only on incomplete images
    serial = None


class TargetWaypointSerial(Node):
    def __init__(self):
        super().__init__('target_waypoint_serial')
        self.declare_parameter('port', '/dev/ttyUSB2')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('read_timeout_s', 0.05)
        self.declare_parameter('poll_period_s', 0.05)
        self.declare_parameter('reconnect_period_s', 2.0)
        self.declare_parameter('max_line_length', 512)

        self._port_name = str(self.get_parameter('port').value)
        self._baud = int(self.get_parameter('baud').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._timeout = float(self.get_parameter('read_timeout_s').value)
        self._reconnect_period = float(
            self.get_parameter('reconnect_period_s').value)
        self._max_line_length = int(self.get_parameter('max_line_length').value)
        self._serial = None
        self._last_open_attempt = 0.0

        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._fix_pub = self.create_publisher(NavSatFix, '/target/fix', q)
        self._geo_pub = self.create_publisher(
            GeoPoseStamped, '/target/geopose', q)
        self._geo_path_pub = self.create_publisher(
            GeoPath, '/target/geopath', q)
        self.create_timer(float(self.get_parameter('poll_period_s').value),
                          self._poll)
        self.get_logger().info(
            f'target waypoint serial: {self._port_name} @ {self._baud}; '
            'accepting TARGET,lat,lon and plain CSV')

    def _open_if_needed(self):
        if serial is None:
            self.get_logger().error(
                'python3-serial is not installed; cannot read target port')
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        if self._serial is not None and self._serial.is_open:
            return True
        if now - self._last_open_attempt < self._reconnect_period:
            return False
        self._last_open_attempt = now
        try:
            self._serial = serial.Serial(self._port_name, self._baud,
                                         timeout=self._timeout)
            self.get_logger().info(f'opened target port {self._port_name}')
            return True
        except Exception as exc:
            self._serial = None
            self.get_logger().warn(f'cannot open {self._port_name}: {exc}')
            return False

    def _poll(self):
        if not self._open_if_needed():
            return
        try:
            while self._serial.in_waiting:
                raw = self._serial.readline(self._max_line_length)
                if not raw:
                    break
                try:
                    line = raw.decode('utf-8', errors='replace').strip()
                except AttributeError:
                    line = str(raw).strip()
                if line:
                    self._handle_line(line)
        except Exception as exc:
            self.get_logger().warn(f'target serial read failed: {exc}')
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    @staticmethod
    def _parse_line(line: str):
        """Parse one line into a single ``(lat, lon, alt)`` point.

        Returns ``None`` when the line is not a recognised single waypoint.
        Multi-waypoint lines are handled by ``_parse_multi_line`` instead.
        """
        line = line.split('*', 1)[0].strip()
        if not line:
            return None
        if line.startswith('{'):
            value = json.loads(line)
            if 'waypoints' in value:
                return None
            lat = value.get('latitude', value.get('lat'))
            lon = value.get('longitude', value.get('lon'))
            alt = value.get('altitude', value.get('alt', 0.0))
            return float(lat), float(lon), float(alt)

        fields = [x.strip() for x in re.split(r'[,;\s]+', line) if x.strip()]
        if not fields:
            return None
        head = fields[0].upper()
        if head == 'WAYPOINTS':
            return None
        if head in {'TARGET', 'WAYPOINT', 'GOTO', 'GOAL'}:
            fields = fields[1:]
        if len(fields) < 2:
            return None
        return (float(fields[0]), float(fields[1]),
                float(fields[2]) if len(fields) > 2 else 0.0)

    @staticmethod
    def _parse_multi_line(line: str):
        """Parse one line into a list of ``(lat, lon, alt)`` waypoints.

        Returns ``None`` when the line is not a recognised multi-waypoint
        route (``WAYPOINTS,lat,lon,lat,lon,...`` or ``{"waypoints":[...]}``).
        """
        line = line.split('*', 1)[0].strip()
        if not line:
            return None
        if line.startswith('{'):
            value = json.loads(line)
            if 'waypoints' not in value:
                return None
            points = []
            for wp in value['waypoints']:
                lat = wp.get('latitude', wp.get('lat'))
                lon = wp.get('longitude', wp.get('lon'))
                alt = wp.get('altitude', wp.get('alt', 0.0))
                points.append((float(lat), float(lon), float(alt)))
            return points

        fields = [x.strip() for x in re.split(r'[,;\s]+', line) if x.strip()]
        if not fields:
            return None
        head = fields[0].upper()
        if head != 'WAYPOINTS':
            return None
        if len(fields) < 5 or (len(fields) - 1) % 2 != 0:
            return None
        points = []
        for i in range(1, len(fields), 2):
            points.append((float(fields[i]), float(fields[i + 1]), 0.0))
        return points

    def _handle_line(self, line: str):
        try:
            multi = self._parse_multi_line(line)
            if multi is not None:
                for lat, lon, alt in multi:
                    if not validate_lat_lon(lat, lon) or not math.isfinite(alt):
                        raise ValueError('latitude/longitude/altitude out of range')
                self._publish_geo_path(multi)
                return
            parsed = self._parse_line(line)
            if parsed is None:
                return
            lat, lon, alt = parsed
            if not validate_lat_lon(lat, lon) or not math.isfinite(alt):
                raise ValueError('latitude/longitude/altitude out of range')
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'ignore target line {line!r}: {exc}')
            return

        stamp = self.get_clock().now().to_msg()
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = self._frame_id
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt
        # This is a command waypoint, not a measurement.  Zero covariance
        # would be interpreted as an unrealistically perfect GNSS fix.
        fix.position_covariance = [1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 4.0]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self._fix_pub.publish(fix)

        geo = GeoPoseStamped()
        geo.header.stamp = stamp
        geo.header.frame_id = 'wgs84'
        geo.pose.position.latitude = lat
        geo.pose.position.longitude = lon
        geo.pose.position.altitude = alt
        geo.pose.orientation = yaw_to_quaternion(0.0)
        self._geo_pub.publish(geo)
        self.get_logger().info(
            f'target waypoint: lat={lat:.8f}, lon={lon:.8f}, alt={alt:.2f}')

    def _publish_geo_path(self, points: List[Tuple[float, float, float]]):
        """Publish a multi-waypoint route as a GeoPath on /target/geopath."""
        stamp = self.get_clock().now().to_msg()
        path = GeoPath()
        path.header.stamp = stamp
        path.header.frame_id = 'wgs84'
        for lat, lon, alt in points:
            gp = GeoPoseStamped()
            gp.header.stamp = stamp
            gp.header.frame_id = 'wgs84'
            gp.pose.position.latitude = lat
            gp.pose.position.longitude = lon
            gp.pose.position.altitude = alt
            gp.pose.orientation = yaw_to_quaternion(0.0)
            path.poses.append(gp)
        self._geo_path_pub.publish(path)
        self.get_logger().info(
            f'target waypoint route: {len(points)} point(s) on /target/geopath')


def main(args=None):
    rclpy.init(args=args)
    node = TargetWaypointSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._serial is not None:
            node._serial.close()
        node.destroy_node()
        rclpy.shutdown()
