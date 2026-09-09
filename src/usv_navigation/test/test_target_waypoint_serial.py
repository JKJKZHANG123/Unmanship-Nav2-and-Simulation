"""Tests for the accepted target serial line formats."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from usv_navigation.target_waypoint_serial import TargetWaypointSerial


def test_parse_prefixed_csv():
    assert TargetWaypointSerial._parse_line(
        'TARGET,31.230416,121.473701') == (
            31.230416, 121.473701, 0.0)


def test_parse_whitespace_with_altitude():
    assert TargetWaypointSerial._parse_line(
        '31.230416 121.473701 4.5') == (
            31.230416, 121.473701, 4.5)


def test_parse_json_aliases():
    assert TargetWaypointSerial._parse_line(
        '{"lat":31.23,"lon":121.47,"alt":2}') == (31.23, 121.47, 2.0)


def test_parse_multi_csv():
    assert TargetWaypointSerial._parse_multi_line(
        'WAYPOINTS,31.23,121.47,31.24,121.48') == [
            (31.23, 121.47, 0.0), (31.24, 121.48, 0.0)]


def test_parse_multi_json():
    assert TargetWaypointSerial._parse_multi_line(
        '{"waypoints":[{"lat":31.23,"lon":121.47},'
        '{"lat":31.24,"lon":121.48,"alt":3.0}]}') == [
            (31.23, 121.47, 0.0), (31.24, 121.48, 3.0)]


def test_parse_multi_rejects_single():
    # A single-waypoint line must not be parsed as a multi-waypoint route.
    assert TargetWaypointSerial._parse_multi_line('TARGET,31.23,121.47') is None
    assert TargetWaypointSerial._parse_multi_line('31.23 121.47') is None
