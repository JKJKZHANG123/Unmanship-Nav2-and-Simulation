"""Unit tests for the dependency-light MAVLink framing helpers."""

import struct

from usv_navigation.mavlink_rtk_bridge import (
    _crc_x25,
    _encode_mission_count,
    _encode_mission_item,
    _encode_mission_item_int,
    MavlinkParser,
)


def _heartbeat_packet(seq=1, sysid=42, compid=1):
    # MAVLink HEARTBEAT payload: custom_mode, type, autopilot, base_mode,
    # system_status, mavlink_version.
    payload = struct.pack('<IBBBBB', 0, 2, 3, 0, 4, 3)
    header = bytes([len(payload), seq, sysid, compid, 0])
    crc = _crc_x25(header + payload + bytes([50]))
    return b'\xfe' + header + payload + struct.pack('<H', crc)


def test_parser_accepts_mavlink_v1_heartbeat():
    messages = MavlinkParser().feed(b'noise' + _heartbeat_packet())
    assert len(messages) == 1
    msg_id, payload, sysid, compid = messages[0]
    assert msg_id == 0
    assert len(payload) == 9
    assert (sysid, compid) == (42, 1)


def test_parser_recovers_after_bad_crc():
    bad = bytearray(_heartbeat_packet(seq=2))
    bad[-1] ^= 0xFF
    messages = MavlinkParser().feed(bytes(bad) + _heartbeat_packet(seq=3))
    assert len(messages) == 1
    assert messages[0][0] == 0


def _gps_raw_v2_packet():
    payload = bytearray(52)
    struct.pack_into('<q', payload, 0, 123456789)
    struct.pack_into('<i', payload, 8, int(31.2 * 1e7))
    struct.pack_into('<i', payload, 12, int(121.5 * 1e7))
    struct.pack_into('<i', payload, 16, 123450)
    payload[28] = 5
    payload[29] = 20
    struct.pack_into('<I', payload, 34, 25)
    struct.pack_into('<I', payload, 38, 40)
    struct.pack_into('<H', payload, 50, 9000)
    header = bytes([len(payload), 0, 0, 7, 42, 1, 24, 0, 0])
    crc = _crc_x25(header + payload + bytes([24]))
    return b'\xfd' + header + payload + struct.pack('<H', crc)


def test_parser_accepts_mavlink_v2_gps_raw_int():
    messages = MavlinkParser().feed(_gps_raw_v2_packet())
    assert len(messages) == 1
    assert messages[0][0] == 24
    assert messages[0][1][28] == 5
    assert struct.unpack_from('<H', messages[0][1], 50)[0] == 9000


def test_mission_count_frame_is_valid_and_contains_target():
    packet = _encode_mission_count(
        seq=9, source_system=255, source_component=190,
        target_system=1, target_component=1, count=12)
    messages = MavlinkParser().feed(packet)
    assert len(messages) == 1
    msg_id, payload, sysid, compid = messages[0]
    assert (msg_id, sysid, compid) == (44, 255, 190)
    assert struct.unpack_from('<H', payload, 0)[0] == 12
    assert payload[2:5] == bytes([1, 1, 0])


def test_mission_item_int_uses_e7_coordinates_and_compass_yaw():
    packet = _encode_mission_item_int(
        seq=10, source_system=255, source_component=190,
        target_system=1, target_component=1, item_seq=2,
        latitude=31.230416, longitude=121.473701, altitude=0.0,
        yaw_deg=90.0, acceptance_radius_m=1.5)
    messages = MavlinkParser().feed(packet)
    assert len(messages) == 1
    msg_id, payload, _, _ = messages[0]
    assert msg_id == 73
    assert struct.unpack_from('<i', payload, 16)[0] == 312304160
    assert struct.unpack_from('<i', payload, 20)[0] == 1214737010
    assert struct.unpack_from('<f', payload, 4)[0] == 1.5
    assert struct.unpack_from('<H', payload, 28)[0] == 2
    assert struct.unpack_from('<H', payload, 30)[0] == 16
    assert payload[32:38] == bytes([1, 1, 3, 0, 1, 0])


def test_legacy_mission_request_uses_float_coordinate_item():
    packet = _encode_mission_item(
        seq=11, source_system=255, source_component=190,
        target_system=1, target_component=1, item_seq=2,
        latitude=31.230416, longitude=121.473701, altitude=0.0,
        yaw_deg=90.0, acceptance_radius_m=1.5)
    messages = MavlinkParser().feed(packet)
    assert len(messages) == 1
    msg_id, payload, _, _ = messages[0]
    assert msg_id == 39
    assert abs(struct.unpack_from('<f', payload, 16)[0] - 31.230416) < 1e-5
    assert abs(struct.unpack_from('<f', payload, 20)[0] - 121.473701) < 1e-5
    assert struct.unpack_from('<H', payload, 28)[0] == 2
    assert struct.unpack_from('<H', payload, 30)[0] == 16
    assert payload[32:38] == bytes([1, 1, 3, 0, 1, 0])


def test_mission_item_int_uses_requested_frame():
    packet = _encode_mission_item_int(
        seq=12, source_system=255, source_component=190,
        target_system=1, target_component=1, item_seq=0,
        latitude=31.0, longitude=121.0, altitude=2.0, yaw_deg=0.0,
        acceptance_radius_m=1.0, frame=6)
    messages = MavlinkParser().feed(packet)
    assert len(messages) == 1
    payload = messages[0][1]
    assert payload[34] == 6


def test_geo_yaw_conversion_convention():
    # The mission encoder receives compass degrees.  Check the expected
    # cardinal values independently of ROS message construction.
    from usv_navigation.mavlink_rtk_bridge import MavlinkRtkBridge
    from geometry_msgs.msg import Pose

    pose = Pose()
    # ENU yaw 0: east, therefore compass heading 90 degrees.
    assert abs(MavlinkRtkBridge._geo_yaw(pose) - 90.0) < 1e-6
    # ENU yaw +pi/2: north, therefore compass heading 0 degrees.
    pose.orientation.z = 2.0 ** -0.5
    pose.orientation.w = 2.0 ** -0.5
    assert abs(MavlinkRtkBridge._geo_yaw(pose) - 0.0) < 1e-6
