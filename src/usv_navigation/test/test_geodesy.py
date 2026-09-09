"""Tests for the dependency-free WGS84/UTM conversions."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from usv_navigation.geodesy import lat_lon_to_utm, utm_to_lat_lon


def test_wgs84_utm_round_trip_northern_hemisphere():
    lat, lon = 31.230416, 121.473701
    easting, northing, zone, northern = lat_lon_to_utm(lat, lon)
    result_lat, result_lon = utm_to_lat_lon(easting, northing, zone, northern)
    assert abs(result_lat - lat) < 1e-6
    assert abs(result_lon - lon) < 1e-6


def test_wgs84_utm_round_trip_southern_hemisphere():
    lat, lon = -33.7, 150.67
    easting, northing, zone, northern = lat_lon_to_utm(lat, lon)
    result_lat, result_lon = utm_to_lat_lon(easting, northing, zone, northern)
    assert abs(result_lat - lat) < 1e-6
    assert abs(result_lon - lon) < 1e-6
