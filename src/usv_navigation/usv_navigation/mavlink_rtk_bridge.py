#!/usr/bin/env python3
"""Bridge flight-controller MAVLink GPS/heading data to ROS 2.

The expected wiring is a flight-controller MAVLink telemetry UART connected to
one of the Jetson's 3.3 V GPIO UARTs.  This node deliberately implements the
small MAVLink subset it needs, so the runtime only depends on pyserial (which
is already a ROS package dependency in this workspace).

Published interfaces:
  /gps/fix            sensor_msgs/NavSatFix
  /gps/heading        std_msgs/Float64, ENU yaw in radians
  /gps/heading_source std_msgs/String
  /gps/status         std_msgs/String containing JSON diagnostics
  /gps/mavlink_raw    std_msgs/String diagnostic mirror of received UART bytes

MAVLink compass headings are clockwise from North; ROS planar yaw is measured
counter-clockwise from East, therefore yaw_enu = pi/2 - heading_compass.
"""

import json
import math
import struct
import time
from typing import Dict, Optional, Tuple

from geographic_msgs.msg import GeoPath
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String


# MAVLink common.xml CRC extras for the messages used here.
_CRC_EXTRA = {
    0: 50,    # HEARTBEAT
    24: 24,   # GPS_RAW_INT
    30: 39,   # ATTITUDE
    33: 104,  # GLOBAL_POSITION_INT
    124: 87,  # GPS2_RAW
    39: 254,  # MISSION_ITEM
    40: 230,  # MISSION_REQUEST
    44: 221,  # MISSION_COUNT
    47: 153,  # MISSION_ACK
    51: 196,  # MISSION_REQUEST_INT
    73: 38,   # MISSION_ITEM_INT
}
_COMMAND_LONG_CRC_EXTRA = 152


def _crc_x25(data: bytes) -> int:
    """Calculate the MAVLink X.25 checksum."""
    crc = 0xFFFF
    for value in data:
        tmp = value ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def _read_u16(payload: bytes, offset: int) -> int:
    return struct.unpack_from('<H', payload, offset)[0]


def _read_u32(payload: bytes, offset: int) -> int:
    return struct.unpack_from('<I', payload, offset)[0]


def _read_i32(payload: bytes, offset: int) -> int:
    return struct.unpack_from('<i', payload, offset)[0]


class MavlinkParser:
    """Incremental MAVLink 1/2 parser for a serial byte stream."""

    def __init__(self):
        self._buffer = bytearray()
        self.frames_seen = 0
        self.valid_frames = 0
        self.crc_errors = 0
        self.unknown_frames = 0
        self.message_counts = {}

    def feed(self, data: bytes):
        self._buffer.extend(data)
        messages = []
        while True:
            if not self._buffer:
                break
            magic_positions = [p for p in (
                self._buffer.find(b'\xfe'), self._buffer.find(b'\xfd'))
                if p >= 0]
            if not magic_positions:
                self._buffer.clear()
                break
            start = min(magic_positions)
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 2:
                break
            magic = self._buffer[0]
            payload_len = self._buffer[1]
            header_len = 6 if magic == 0xFE else 10
            if magic not in (0xFE, 0xFD):
                del self._buffer[0]
                continue
            if len(self._buffer) < header_len:
                break
            signed = magic == 0xFD and (self._buffer[2] & 0x01) != 0
            total_len = header_len + payload_len + 2 + (13 if signed else 0)
            if len(self._buffer) < total_len:
                break

            self.frames_seen += 1
            packet = bytes(self._buffer[:total_len])
            if magic == 0xFE:
                msg_id = packet[5]
                header = packet[1:6]
                payload_start = 6
            else:
                msg_id = int.from_bytes(packet[7:10], 'little')
                header = packet[1:10]
                payload_start = 10
            payload = packet[payload_start:payload_start + payload_len]
            received_crc = int.from_bytes(
                packet[payload_start + payload_len:payload_start + payload_len + 2],
                'little')
            extra = _CRC_EXTRA.get(msg_id)
            if extra is None:
                # Unknown messages are irrelevant to this bridge. Discard the
                # frame without blocking known messages behind it.
                self.unknown_frames += 1
                del self._buffer[:total_len]
                continue
            calculated_crc = _crc_x25(header + payload + bytes([extra]))
            if calculated_crc != received_crc:
                self.crc_errors += 1
                # A corrupted byte can make the declared length meaningless;
                # rescan from the next byte for the next valid frame.
                del self._buffer[0]
                continue
            self.valid_frames += 1
            self.message_counts[msg_id] = self.message_counts.get(msg_id, 0) + 1
            del self._buffer[:total_len]
            messages.append((msg_id, payload, packet[3] if magic == 0xFE else packet[5],
                             packet[4] if magic == 0xFE else packet[6]))
        return messages


def _encode_v2_message(seq: int, source_system: int, source_component: int,
                       msg_id: int, payload: bytes, crc_extra: int) -> bytes:
    """Encode one unsigned MAVLink 2 frame."""
    if len(payload) > 255:
        raise ValueError('MAVLink payload cannot exceed 255 bytes')
    header = bytes([len(payload), 0, 0, seq & 0xFF,
                    source_system & 0xFF, source_component & 0xFF])
    header += int(msg_id).to_bytes(3, 'little')
    crc = _crc_x25(header + payload + bytes([crc_extra]))
    return b'\xfd' + header + payload + struct.pack('<H', crc)


def _encode_command_long(seq: int, source_system: int, source_component: int,
                         target_system: int, target_component: int,
                         command: int, param1: float, param2: float) -> bytes:
    """Encode MAV_CMD_SET_MESSAGE_INTERVAL as a MAVLink 2 COMMAND_LONG."""
    payload = struct.pack(
        '<7fHBBB', param1, param2, 0.0, 0.0, 0.0, 0.0, 0.0,
        command, target_system, target_component, 0)
    return _encode_v2_message(
        seq, source_system, source_component, 76, payload,
        _COMMAND_LONG_CRC_EXTRA)


def _encode_mission_count(seq: int, source_system: int, source_component: int,
                          target_system: int, target_component: int,
                          count: int, mission_type: int = 0) -> bytes:
    payload = struct.pack(
        '<HBBB', count, target_system, target_component, mission_type)
    return _encode_v2_message(
        seq, source_system, source_component, 44, payload, 221)


def _encode_mission_item_int(
        seq: int, source_system: int, source_component: int,
        target_system: int, target_component: int, item_seq: int,
        latitude: float, longitude: float, altitude: float, yaw_deg: float,
        acceptance_radius_m: float, mission_type: int = 0,
        frame: int = 3) -> bytes:
    # MAVLink MISSION_ITEM_INT wire order.  For global frames x/y are
    # latitude/longitude in degrees * 1e7 and z is altitude in metres.
    payload = struct.pack(
        '<ffffiifHHBBBBBB',
        0.0, acceptance_radius_m, 0.0, yaw_deg,
        int(round(latitude * 1e7)), int(round(longitude * 1e7)),
        float(altitude), item_seq, 16, target_system, target_component,
        frame, 0, 1, mission_type)
    return _encode_v2_message(
        seq, source_system, source_component, 73, payload, 38)


def _encode_mission_item(
        seq: int, source_system: int, source_component: int,
        target_system: int, target_component: int, item_seq: int,
        latitude: float, longitude: float, altitude: float, yaw_deg: float,
        acceptance_radius_m: float, mission_type: int = 0,
        frame: int = 3) -> bytes:
    """Encode the float-coordinate form used after MISSION_REQUEST."""
    # MISSION_ITEM is the legacy counterpart to MISSION_ITEM_INT.  It is
    # required when a vehicle explicitly requests message 40 rather than 51.
    payload = struct.pack(
        '<fffffffHHBBBBBB',
        0.0, acceptance_radius_m, 0.0, yaw_deg,
        float(latitude), float(longitude), float(altitude), item_seq, 16,
        target_system, target_component, frame, 0, 1, mission_type)
    return _encode_v2_message(
        seq, source_system, source_component, 39, payload, 254)


class MavlinkRtkBridge(Node):
    """Read MAVLink GPS/heading messages and publish ROS sensor topics."""

    def __init__(self):
        super().__init__('mavlink_rtk_bridge')
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('heading_topic', '/gps/heading')
        self.declare_parameter('heading_source_topic', '/gps/heading_source')
        self.declare_parameter('status_topic', '/gps/status')
        self.declare_parameter('raw_topic', '/gps/mavlink_raw')
        self.declare_parameter('publish_raw_serial', True)
        self.declare_parameter('raw_publish_max_bytes', 512)
        self.declare_parameter('request_stream', True)
        self.declare_parameter('stream_rate_hz', 10.0)
        self.declare_parameter('reconnect_period_s', 2.0)
        self.declare_parameter('stale_timeout_s', 2.0)
        self.declare_parameter('heading_fallback_timeout_s', 2.0)
        self.declare_parameter('heading_offset_deg', 0.0)
        self.declare_parameter('source_system', 255)
        self.declare_parameter('source_component', 190)
        self.declare_parameter('target_system', 0)
        self.declare_parameter('target_component', 0)
        self.declare_parameter('default_horizontal_accuracy_m', 2.0)
        self.declare_parameter('default_vertical_accuracy_m', 3.0)
        # Mission upload is opt-in. It changes the vehicle's onboard mission,
        # but never sends thrust, steering, arming, or mode commands.
        self.declare_parameter('mission_upload_enabled', False)
        self.declare_parameter('mission_path_topic', '/plan_geodetic')
        self.declare_parameter('mission_max_points', 100)
        self.declare_parameter('mission_min_spacing_m', 1.0)
        self.declare_parameter('mission_acceptance_radius_m', 1.0)
        self.declare_parameter(
            'mission_frame', 3)  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.declare_parameter('mission_altitude_m', 0.0)
        self.declare_parameter('mission_use_path_altitude', False)
        self.declare_parameter('mission_timeout_s', 3.0)
        self.declare_parameter('mission_max_retries', 3)

        self._port = str(self.get_parameter('port').value)
        self._baud = int(self.get_parameter('baud').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._request_stream = bool(self.get_parameter('request_stream').value)
        self._raw_topic = str(self.get_parameter('raw_topic').value)
        self._publish_raw_serial = bool(
            self.get_parameter('publish_raw_serial').value)
        self._raw_publish_max_bytes = max(0, int(
            self.get_parameter('raw_publish_max_bytes').value))
        self._stream_rate = float(self.get_parameter('stream_rate_hz').value)
        self._reconnect_period = float(self.get_parameter('reconnect_period_s').value)
        self._stale_timeout = float(self.get_parameter('stale_timeout_s').value)
        self._heading_fallback_timeout = float(
            self.get_parameter('heading_fallback_timeout_s').value)
        self._heading_offset = float(self.get_parameter('heading_offset_deg').value)
        self._source_system = int(self.get_parameter('source_system').value)
        self._source_component = int(self.get_parameter('source_component').value)
        self._target_system = int(self.get_parameter('target_system').value)
        self._target_component = int(self.get_parameter('target_component').value)
        self._default_hacc = float(
            self.get_parameter('default_horizontal_accuracy_m').value)
        self._default_vacc = float(
            self.get_parameter('default_vertical_accuracy_m').value)
        self._mission_enabled = bool(
            self.get_parameter('mission_upload_enabled').value)
        self._mission_topic = str(
            self.get_parameter('mission_path_topic').value)
        self._mission_max_points = max(2, int(
            self.get_parameter('mission_max_points').value))
        self._mission_min_spacing = max(0.0, float(
            self.get_parameter('mission_min_spacing_m').value))
        self._mission_acceptance_radius = max(0.0, float(
            self.get_parameter('mission_acceptance_radius_m').value))
        self._mission_frame = int(self.get_parameter('mission_frame').value)
        if not 0 <= self._mission_frame <= 255:
            raise ValueError('mission_frame must be in [0, 255]')
        self._mission_altitude = float(
            self.get_parameter('mission_altitude_m').value)
        self._mission_use_path_altitude = bool(
            self.get_parameter('mission_use_path_altitude').value)
        self._mission_timeout = max(0.5, float(
            self.get_parameter('mission_timeout_s').value))
        self._mission_max_retries = max(0, int(
            self.get_parameter('mission_max_retries').value))

        # GPS and heading are low-rate state topics consumed by reliable
        # subscribers (target_to_goal, geodetic_path_publisher, and
        # navsat_transform).  A BEST_EFFORT publisher would not match those
        # subscribers in DDS, so keep this interface RELIABLE.
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self._fix_pub = self.create_publisher(NavSatFix, '/gps/fix', qos)
        self._heading_pub = self.create_publisher(
            Float64, str(self.get_parameter('heading_topic').value), qos)
        self._source_pub = self.create_publisher(
            String, str(self.get_parameter('heading_source_topic').value), qos)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), qos)
        self._raw_pub = self.create_publisher(String, self._raw_topic, qos)
        if self._mission_enabled:
            mission_qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(
                GeoPath, self._mission_topic, self._on_geodetic_path,
                mission_qos)

        self._serial = None
        self._parser = MavlinkParser()
        self._last_open_attempt = 0.0
        self._last_rx = 0.0
        self._last_fix = 0.0
        self._last_status = 0.0
        self._last_stream_request = 0.0
        self._stream_requested = False
        self._mav_seq = 0
        self._system_id = None
        self._component_id = None
        self._last_fix_data: Optional[Dict] = None
        self._raw_bytes_rx = 0
        self._last_heading: Optional[Tuple[float, str]] = None
        self._last_primary_fix = 0.0
        self._vehicle_system_id = None
        self._vehicle_component_id = None
        self._mission_pending = None
        self._mission_active = None
        self._mission_received_signature = None
        self._mission_completed_signature = None
        self._mission_last_tx = 0.0
        self._mission_retries = 0
        self._mission_last_request = None

        self.create_timer(0.02, self._poll_serial)
        self.create_timer(1.0, self._publish_status)
        self.create_timer(0.1, self._mission_tick)
        self.get_logger().info(
            f'MAVLink RTK bridge configured for {self._port} @ {self._baud}; '
            'expects 3.3 V UART MAVLink from the flight controller')
        if self._mission_enabled:
            self.get_logger().warn(
                f'MAVLink mission upload ENABLED from {self._mission_topic}; '
                'this updates the FC mission but does not command motion')

    def _open_serial(self):
        now = time.monotonic()
        if now - self._last_open_attempt < self._reconnect_period:
            return
        self._last_open_attempt = now
        try:
            import serial
            self._serial = serial.Serial(
                self._port, self._baud, timeout=0.01, write_timeout=0.2)
            self._parser = MavlinkParser()
            self._stream_requested = False
            self._last_stream_request = 0.0
            self.get_logger().info(f'opened MAVLink UART {self._port}')
        except Exception as exc:
            self._serial = None
            self.get_logger().warn(f'cannot open MAVLink UART {self._port}: {exc}')

    def _close_serial(self):
        # A newer replan may already be queued while the FC is processing the
        # previous mission.  Never replace that newer path with stale data.
        if (self._mission_active is not None and
                self._mission_pending is None):
            self._mission_pending = (
                self._mission_active['signature'],
                self._mission_active['points'],
            )
        elif self._mission_active is not None:
            self.get_logger().warn(
                'serial disconnected during mission upload; retaining the '
                'newer queued path')
        self._mission_active = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        self._stream_requested = False
        # Require a fresh heartbeat after reconnect before requesting streams
        # or starting a queued mission.  Do not reuse a stale FC identity.
        self._system_id = None
        self._component_id = None
        self._vehicle_system_id = None
        self._vehicle_component_id = None
        self._mission_last_tx = 0.0
        self._mission_last_request = None
        self._mission_retries = 0

    def _poll_serial(self):
        if self._serial is None:
            self._open_serial()
            return
        try:
            waiting = int(getattr(self._serial, 'in_waiting', 0))
            data = self._serial.read(max(1, min(waiting, 4096)))
            if data:
                self._raw_bytes_rx += len(data)
                if self._publish_raw_serial:
                    preview = data[:self._raw_publish_max_bytes]
                    suffix = ' ...' if len(preview) < len(data) else ''
                    self._raw_pub.publish(String(data=(
                        f'bytes={len(data)} total={self._raw_bytes_rx} '
                        f'hex={preview.hex(" ")}{suffix}')))
                messages = self._parser.feed(data)
                if messages:
                    self._last_rx = time.monotonic()
                for msg_id, payload, sysid, compid in messages:
                    self._handle_message(msg_id, payload, sysid, compid)
            if (self._request_stream and self._system_id is not None and
                    (not self._stream_requested or
                     time.monotonic() - self._last_stream_request > 10.0)):
                self._request_message_intervals()
        except Exception as exc:
            self.get_logger().warn(f'MAVLink UART read failed: {exc}')
            self._close_serial()

    @staticmethod
    def _geo_yaw(pose):
        """Convert ROS ENU yaw to MAVLink's compass heading in degrees.

        ROS planar yaw is measured counter-clockwise from East.  MAVLink
        waypoint yaw is a compass heading, clockwise from North.  Sending
        the ROS value directly makes an eastbound path look northbound to the
        flight controller.
        """
        q = pose.orientation
        yaw_enu = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        compass_deg = (90.0 - math.degrees(yaw_enu)) % 360.0
        # Avoid returning 360.0 from tiny negative floating-point residue
        # when the mathematically expected heading is north (0 degrees).
        return 0.0 if math.isclose(compass_deg, 360.0, abs_tol=1e-9) else compass_deg

    def _on_geodetic_path(self, msg: GeoPath):
        """Queue a latest-wins WGS84 path for MAVLink mission upload."""
        if not self._mission_enabled:
            return
        if not msg.poses:
            # A cleared route: drop any queued-but-not-yet-uploaded path so a
            # stale mission is not pushed after a cancel.  An in-flight upload
            # is left to finish rather than interrupted mid-sequence (interrupting
            # would leave the FC mission half-written).  Clearing the FC mission
            # itself would require MISSION_CLEAR_ALL and is intentionally not
            # sent here.
            if self._mission_pending is not None:
                self.get_logger().info(
                    'cleared queued MAVLink mission (empty /plan_geodetic)')
                self._mission_pending = None
            return
        points = []
        last = None
        for pose in msg.poses:
            lat = float(pose.pose.position.latitude)
            lon = float(pose.pose.position.longitude)
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                continue
            if not (math.isfinite(lat) and math.isfinite(lon)):
                continue
            if (last is not None and self._mission_min_spacing > 0.0 and
                    self._approx_geodesic_distance(last[0], last[1], lat, lon)
                    < self._mission_min_spacing):
                continue
            altitude = float(pose.pose.position.altitude)
            if not self._mission_use_path_altitude or not math.isfinite(altitude):
                altitude = self._mission_altitude
            points.append((lat, lon, altitude, self._geo_yaw(pose.pose)))
            last = (lat, lon)
        if len(points) < 2:
            self.get_logger().warn(
                'geodetic path has fewer than two valid mission points')
            return
        if len(points) > self._mission_max_points:
            # Preserve both endpoints while bounding the FC mission size.
            indices = [round(i * (len(points) - 1) /
                             (self._mission_max_points - 1))
                       for i in range(self._mission_max_points)]
            points = [points[i] for i in indices]
        # Include heading/altitude as well as position.  A path can retain the
        # same XY samples while its desired yaw changes after replanning.
        signature = tuple((round(p[0], 7), round(p[1], 7),
                           round(p[2], 2), round(p[3], 2)) for p in points)
        if signature in (self._mission_received_signature,
                         self._mission_completed_signature):
            return
        self._mission_received_signature = signature
        self._mission_pending = (signature, points)
        self.get_logger().info(
            f'queued {len(points)} geodetic points for MAVLink mission upload')
        self._try_start_mission_upload()

    @staticmethod
    def _approx_geodesic_distance(lat1, lon1, lat2, lon2):
        # Accurate enough for path decimation over normal USV mission ranges.
        lat_scale = 111320.0
        lon_scale = 111320.0 * math.cos(math.radians((lat1 + lat2) * 0.5))
        return math.hypot((lat2 - lat1) * lat_scale,
                          (lon2 - lon1) * lon_scale)

    def _mission_target(self):
        return (self._target_system or self._vehicle_system_id or
                self._system_id or 1,
                self._target_component or self._vehicle_component_id or
                self._component_id or 1)

    def _write_mavlink(self, packet):
        if self._serial is None:
            return False
        try:
            self._serial.write(packet)
            self._serial.flush()
            self._mission_last_tx = time.monotonic()
            return True
        except Exception as exc:
            self.get_logger().warn(f'MAVLink write failed: {exc}')
            self._close_serial()
            return False

    def _try_start_mission_upload(self):
        if (not self._mission_enabled or self._mission_active is not None or
                self._mission_pending is None or self._serial is None or
                self._system_id is None):
            return
        signature, points = self._mission_pending
        target_system, target_component = self._mission_target()
        self._mission_pending = None
        self._mission_active = {
            'signature': signature,
            'points': points,
            'target_system': target_system,
            'target_component': target_component,
        }
        self._mission_retries = 0
        self._mission_last_request = None
        self._mission_last_tx = 0.0
        self._send_mission_count()

    def _send_mission_count(self):
        if self._mission_active is None:
            return
        active = self._mission_active
        packet = _encode_mission_count(
            self._mav_seq, self._source_system, self._source_component,
            active['target_system'], active['target_component'],
            len(active['points']))
        self._mav_seq = (self._mav_seq + 1) & 0xFF
        if self._write_mavlink(packet):
            self.get_logger().info(
                f'sent MISSION_COUNT={len(active["points"])}; waiting for '
                'MISSION_REQUEST(_INT)')

    def _send_mission_item(self, item_seq, request_msg_id=51):
        if self._mission_active is None:
            return
        active = self._mission_active
        if not 0 <= item_seq < len(active['points']):
            self.get_logger().error(
                f'FC requested invalid mission sequence {item_seq}')
            return
        lat, lon, altitude, yaw_deg = active['points'][item_seq]
        encoder = (_encode_mission_item_int if request_msg_id == 51
                   else _encode_mission_item)
        packet = encoder(
            self._mav_seq, self._source_system, self._source_component,
            active['target_system'], active['target_component'], item_seq,
            lat, lon, altitude, yaw_deg, self._mission_acceptance_radius,
            frame=self._mission_frame)
        self._mav_seq = (self._mav_seq + 1) & 0xFF
        if self._write_mavlink(packet):
            self._mission_last_request = item_seq
            self.get_logger().info(
                f'sent MISSION_ITEM_INT seq={item_seq + 1}/'
                f'{len(active["points"])}')

    def _handle_mission_request(self, payload, sysid, compid, request_msg_id):
        if self._mission_active is None or len(payload) < 4:
            return
        active = self._mission_active
        if (sysid != active['target_system'] or
                compid != active['target_component']):
            return
        # MISSION_REQUEST(_INT) serializes seq, target_system,
        # target_component, mission_type. Only answer requests addressed to
        # this bridge and for the waypoint mission type.
        if (payload[2] != self._source_system or
                payload[3] != self._source_component or
                (len(payload) >= 5 and payload[4] != 0)):
            return
        item_seq = _read_u16(payload, 0)
        self._send_mission_item(item_seq, request_msg_id=request_msg_id)

    def _handle_mission_ack(self, payload, sysid, compid):
        if self._mission_active is None or len(payload) < 3:
            return
        active = self._mission_active
        if (sysid != active['target_system'] or
                compid != active['target_component']):
            return
        # Base MAVLink MISSION_ACK payload is target_system, target_component,
        # type. Extensions may append mission_type and opaque_id.
        if (payload[0] != self._source_system or
                payload[1] != self._source_component or
                (len(payload) >= 4 and payload[3] != 0)):
            return
        ack_type = int(payload[2])
        active = self._mission_active
        self._mission_active = None
        if ack_type == 0:  # MAV_MISSION_ACCEPTED
            self._mission_completed_signature = active['signature']
            self.get_logger().info(
                f'FC accepted MAVLink mission ({len(active["points"])} points)')
        else:
            self.get_logger().error(
                f'FC rejected MAVLink mission, MISSION_ACK type={ack_type}')
        self._try_start_mission_upload()

    def _mission_tick(self):
        if not self._mission_enabled:
            return
        self._try_start_mission_upload()
        if self._mission_active is None or self._mission_last_tx <= 0.0:
            return
        if time.monotonic() - self._mission_last_tx <= self._mission_timeout:
            return
        if self._mission_retries >= self._mission_max_retries:
            self.get_logger().error(
                'MAVLink mission upload timed out after the configured retry '
                'limit; waiting for a newer path')
            # Do not requeue this same mission: doing so would reset the retry
            # counter on the next timer tick and create an endless loop.
            self._mission_active = None
            self._mission_last_tx = 0.0
            self._mission_last_request = None
            self._mission_retries = 0
            # If replanning queued a newer path, latest-wins still applies.
            self._try_start_mission_upload()
            return
        self._mission_retries += 1
        self.get_logger().warn(
            f'MAVLink mission response timeout; retry '
            f'{self._mission_retries}/{self._mission_max_retries}')
        self._send_mission_count()

    def _request_message_intervals(self):
        target_system = self._target_system or self._system_id or 1
        target_component = self._target_component or self._component_id or 1
        interval_us = int(1_000_000 / max(self._stream_rate, 0.1))
        # GPS_RAW_INT, GPS2_RAW, GLOBAL_POSITION_INT, ATTITUDE, HEARTBEAT.
        for msg_id in (24, 124, 33, 30, 0):
            packet = _encode_command_long(
                self._mav_seq, self._source_system, self._source_component,
                target_system, target_component, 511, float(msg_id),
                float(interval_us))
            self._mav_seq = (self._mav_seq + 1) & 0xFF
            try:
                self._serial.write(packet)
            except Exception as exc:
                self.get_logger().warn(f'cannot request MAVLink streams: {exc}')
                self._close_serial()
                return
        self._stream_requested = True
        self._last_stream_request = time.monotonic()
        self.get_logger().info(
            f'requested MAVLink GPS/heading streams at {self._stream_rate:.1f} Hz')

    def _handle_message(self, msg_id: int, payload: bytes, sysid: int, compid: int):
        if msg_id == 0:
            # Only HEARTBEAT identifies the flight controller.  GPS and
            # attitude messages can originate from another component; using
            # their IDs as the vehicle ID can misaddress mission uploads.
            self._system_id = sysid
            self._component_id = compid
            self._vehicle_system_id = sysid
            self._vehicle_component_id = compid
            return
        if msg_id in (40, 51):
            self._handle_mission_request(payload, sysid, compid, msg_id)
        elif msg_id == 47:
            self._handle_mission_ack(payload, sysid, compid)
        elif msg_id in (24, 124):
            self._handle_gps_raw(payload, msg_id == 124)
        elif msg_id == 33:
            self._handle_global_position(payload)
        elif msg_id == 30:
            self._handle_attitude(payload)

    @staticmethod
    def _fix_status(fix_type: int) -> int:
        if fix_type < 2:
            return NavSatStatus.STATUS_NO_FIX
        if fix_type == 4:
            return NavSatStatus.STATUS_SBAS_FIX
        if fix_type >= 5:
            return NavSatStatus.STATUS_GBAS_FIX
        return NavSatStatus.STATUS_FIX

    def _handle_gps_raw(self, payload: bytes, is_gps2: bool):
        # GPS_RAW_INT and GPS2_RAW share the position fields, but GPS2_RAW
        # inserts dgps_age before the dilution fields and dgps_numch before
        # yaw.  These offsets follow common.xml wire serialization.
        min_len = 35 if is_gps2 else 30
        if len(payload) < min_len:
            return
        fix_offset = 32 if is_gps2 else 28
        satellites_offset = 33 if is_gps2 else 29
        yaw_offset = 35 if is_gps2 else 50
        h_acc_offset = 41 if is_gps2 else 34
        v_acc_offset = 45 if is_gps2 else 38
        fix_type = int(payload[fix_offset])
        satellites = int(payload[satellites_offset])
        latitude = _read_i32(payload, 8) * 1e-7
        longitude = _read_i32(payload, 12) * 1e-7
        altitude = _read_i32(payload, 16) * 1e-3
        if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
            return
        h_acc = (_read_u32(payload, h_acc_offset) * 1e-3
                 if len(payload) >= h_acc_offset + 4 else self._default_hacc)
        v_acc = (_read_u32(payload, v_acc_offset) * 1e-3
                 if len(payload) >= v_acc_offset + 4 else self._default_vacc)
        h_acc = (h_acc if math.isfinite(h_acc) and h_acc > 0.0
                 else self._default_hacc)
        v_acc = (v_acc if math.isfinite(v_acc) and v_acc > 0.0
                 else self._default_vacc)
        data = {
            'source': 'GPS2_RAW' if is_gps2 else 'GPS_RAW_INT',
            'fix_type': fix_type,
            'satellites_visible': satellites,
            'latitude': latitude,
            'longitude': longitude,
            'altitude': altitude,
            'h_acc': h_acc,
            'v_acc': v_acc,
        }
        now = time.monotonic()
        if not is_gps2 or now - self._last_primary_fix > 1.0:
            self._publish_fix(data)
            if not is_gps2:
                self._last_primary_fix = now

        # The moving-baseline heading is normally exposed in GPS_RAW_INT.yaw
        # or GPS2_RAW.yaw, in centidegrees.  0xffff and zero mean unavailable.
        if len(payload) >= yaw_offset + 2:
            raw_yaw = _read_u16(payload, yaw_offset)
            if raw_yaw not in (0, 0xFFFF) and fix_type >= 3:
                compass_deg = raw_yaw * 0.01
                self._publish_compass_heading(compass_deg, data['source'])
        self._last_fix_data = data

    def _handle_global_position(self, payload: bytes):
        if len(payload) < 28:
            return
        raw_hdg = _read_u16(payload, 26)
        if raw_hdg not in (0, 0xFFFF):
            # GLOBAL_POSITION_INT.hdg is centidegrees clockwise from North.
            self._publish_compass_heading(raw_hdg * 0.01, 'GLOBAL_POSITION_INT')

    def _handle_attitude(self, payload: bytes):
        if len(payload) < 16:
            return
        yaw_ned = struct.unpack_from('<f', payload, 12)[0]
        if math.isfinite(yaw_ned):
            # ATTITUDE.yaw is NED yaw: 0 rad points north and positive
            # yaw turns clockwise. Convert it to the same compass heading
            # convention used by GPS_RAW_INT/GLOBAL_POSITION_INT.hdg.
            compass_deg = math.degrees(yaw_ned)
            self._publish_compass_heading(compass_deg, 'ATTITUDE')

    def _publish_fix(self, data: Dict):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.status.status = self._fix_status(data['fix_type'])
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = data['latitude']
        msg.longitude = data['longitude']
        msg.altitude = data['altitude']
        msg.position_covariance[0] = data['h_acc'] ** 2
        msg.position_covariance[4] = data['h_acc'] ** 2
        msg.position_covariance[8] = data['v_acc'] ** 2
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._fix_pub.publish(msg)
        self._last_fix = time.monotonic()

    def _publish_compass_heading(self, compass_deg: float, source: str):
        now = time.monotonic()
        source_priority = {
            'GPS_RAW_INT': 0,
            'GPS2_RAW': 0,
            'GLOBAL_POSITION_INT': 1,
            'ATTITUDE': 2,
        }.get(source, 3)
        # Prefer dual-antenna absolute yaw. A lower-priority fallback may only
        # replace it after it has gone stale; this prevents ATTITUDE messages
        # arriving at a higher rate from continuously overwriting RTK yaw.
        if self._last_heading is not None:
            last_time, last_source = self._last_heading
            last_priority = {
                'GPS_RAW_INT': 0,
                'GPS2_RAW': 0,
                'GLOBAL_POSITION_INT': 1,
                'ATTITUDE': 2,
            }.get(last_source, 3)
            if (source != last_source and source_priority >= last_priority and
                    now - last_time <= self._heading_fallback_timeout):
                return
        compass_deg = (compass_deg + self._heading_offset) % 360.0
        yaw_enu = (math.pi / 2.0 - math.radians(compass_deg) + math.pi) % (
            2.0 * math.pi) - math.pi
        self._heading_pub.publish(Float64(data=yaw_enu))
        self._source_pub.publish(String(data=source))
        self._last_heading = (now, source)

    def _publish_status(self):
        now = time.monotonic()
        data = {
            'connected': self._serial is not None,
            'port': self._port,
            'baud': self._baud,
            'raw_topic': self._raw_topic,
            'raw_bytes_rx': self._raw_bytes_rx,
            'heartbeat_received': self._system_id is not None,
            'system_id': self._system_id,
            'component_id': self._component_id,
            'fix_age_s': (now - self._last_fix) if self._last_fix else None,
            'heading_age_s': (now - self._last_heading[0])
            if self._last_heading else None,
            'heading_source': self._last_heading[1]
            if self._last_heading else None,
            'stale': (not self._last_rx or now - self._last_rx > self._stale_timeout),
            'request_stream': self._request_stream,
            'stream_requested': self._stream_requested,
            'mission_upload_enabled': self._mission_enabled,
            'mission_topic': self._mission_topic,
            'mission_state': ('uploading' if self._mission_active is not None
                              else 'queued' if self._mission_pending is not None
                              else 'idle'),
            'mission_target_system': (
                self._mission_active['target_system']
                if self._mission_active is not None else self._vehicle_system_id),
            'mission_target_component': (
                self._mission_active['target_component']
                if self._mission_active is not None
                else self._vehicle_component_id),
            'mission_points': (
                len(self._mission_active['points'])
                if self._mission_active is not None else
                len(self._mission_pending[1])
                if self._mission_pending is not None else 0),
            'mission_last_request_seq': self._mission_last_request,
            'mission_retries': self._mission_retries,
            'mission_last_tx_age_s': (
                now - self._mission_last_tx
                if self._mission_last_tx > 0.0 else None),
            'fix': self._last_fix_data,
            'parser': {
                'frames_seen': self._parser.frames_seen,
                'valid_frames': self._parser.valid_frames,
                'crc_errors': self._parser.crc_errors,
                'unknown_frames': self._parser.unknown_frames,
                'message_counts': {str(k): v for k, v in
                                   self._parser.message_counts.items()},
            },
        }
        self._status_pub.publish(String(data=json.dumps(data, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkRtkBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._close_serial()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
