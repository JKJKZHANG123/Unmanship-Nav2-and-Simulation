"""Small, dependency-free WGS84/UTM conversion helpers.

The Jetson image does not need pyproj for the short-range boat missions this
package supports.  UTM is used only as a metric intermediate frame; Nav2 still
receives and publishes poses in ``camera_init``.
"""

import math

# WGS84 ellipsoid constants.
_A = 6378137.0
_E2 = 0.0066943799901413165
_EP2 = _E2 / (1.0 - _E2)
_K0 = 0.9996
_FALSE_EASTING = 500000.0
_FALSE_NORTHING = 10000000.0


def validate_lat_lon(latitude: float, longitude: float) -> bool:
    """Return True when latitude/longitude are finite WGS84 coordinates."""
    return (math.isfinite(latitude) and math.isfinite(longitude) and
            -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0)


def utm_zone_for_longitude(longitude: float) -> int:
    """Return the UTM zone number (1..60) containing ``longitude``."""
    if not math.isfinite(longitude):
        raise ValueError('longitude is not finite')
    # Keep +180 in zone 60 rather than producing the invalid zone 61.
    return min(60, max(1, int(math.floor((longitude + 180.0) / 6.0)) + 1))


def lat_lon_to_utm(latitude: float, longitude: float):
    """Convert decimal degrees to ``(easting, northing, zone, northern)``."""
    if not validate_lat_lon(latitude, longitude):
        raise ValueError(f'invalid latitude/longitude: {latitude}, {longitude}')

    zone = utm_zone_for_longitude(longitude)
    lon_origin = (zone - 1) * 6 - 180 + 3
    lat = math.radians(latitude)
    lon = math.radians(longitude)
    lon0 = math.radians(lon_origin)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    tan_lat = math.tan(lat)
    n = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = _EP2 * cos_lat * cos_lat
    a = cos_lat * (lon - lon0)

    m = (_A * ((1 - _E2 / 4 - 3 * _E2**2 / 64 - 5 * _E2**3 / 256) * lat
         - (3 * _E2 / 8 + 3 * _E2**2 / 32 + 45 * _E2**3 / 1024)
         * math.sin(2 * lat)
         + (15 * _E2**2 / 256 + 45 * _E2**3 / 1024) * math.sin(4 * lat)
         - (35 * _E2**3 / 3072) * math.sin(6 * lat)))

    easting = _K0 * n * (a + (1 - t + c) * a**3 / 6
                          + (5 - 18 * t + t**2 + 72 * c - 58 * _EP2)
                          * a**5 / 120) + _FALSE_EASTING
    northing = _K0 * (m + n * tan_lat * (a**2 / 2
                    + (5 - t + 9 * c + 4 * c**2) * a**4 / 24
                    + (61 - 58 * t + t**2 + 600 * c - 330 * _EP2)
                    * a**6 / 720))
    northern = latitude >= 0.0
    if not northern:
        northing += _FALSE_NORTHING
    return easting, northing, zone, northern


def utm_to_lat_lon(easting: float, northing: float, zone: int,
                   northern: bool = True):
    """Convert UTM ``easting/northing`` back to ``(latitude, longitude)``."""
    if not 1 <= int(zone) <= 60:
        raise ValueError(f'invalid UTM zone: {zone}')
    x = float(easting) - _FALSE_EASTING
    y = float(northing)
    if not northern:
        y -= _FALSE_NORTHING

    m = y / _K0
    mu = m / (_A * (1 - _E2 / 4 - 3 * _E2**2 / 64 - 5 * _E2**3 / 256))
    e1 = (1 - math.sqrt(1 - _E2)) / (1 + math.sqrt(1 - _E2))
    j1 = 3 * e1 / 2 - 27 * e1**3 / 32
    j2 = 21 * e1**2 / 16 - 55 * e1**4 / 32
    j3 = 151 * e1**3 / 96
    j4 = 1097 * e1**4 / 512
    fp = (mu + j1 * math.sin(2 * mu) + j2 * math.sin(4 * mu)
          + j3 * math.sin(6 * mu) + j4 * math.sin(8 * mu))

    sin_fp = math.sin(fp)
    cos_fp = math.cos(fp)
    tan_fp = math.tan(fp)
    c1 = _EP2 * cos_fp * cos_fp
    t1 = tan_fp * tan_fp
    n1 = _A / math.sqrt(1 - _E2 * sin_fp * sin_fp)
    r1 = _A * (1 - _E2) / (1 - _E2 * sin_fp * sin_fp)**1.5
    d = x / (n1 * _K0)

    lat = (fp - (n1 * tan_fp / r1) *
           (d**2 / 2 - (5 + 3 * t1 + 10 * c1 - 4 * c1**2 - 9 * _EP2)
            * d**4 / 24 + (61 + 90 * t1 + 298 * c1 + 45 * t1**2
            - 252 * _EP2 - 3 * c1**2) * d**6 / 720))
    lon_origin = (int(zone) - 1) * 6 - 180 + 3
    lon = (math.radians(lon_origin) +
           (d - (1 + 2 * t1 + c1) * d**3 / 6
            + (5 - 2 * c1 + 28 * t1 - 3 * c1**2 + 8 * _EP2
               + 24 * t1**2) * d**5 / 120) / cos_fp)
    return math.degrees(lat), math.degrees(lon)


def yaw_to_quaternion(yaw: float):
    """Return a geometry_msgs Quaternion for an ENU yaw."""
    from geometry_msgs.msg import Quaternion

    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0),
                      w=math.cos(yaw / 2.0))


def quaternion_to_yaw(q) -> float:
    """Extract planar ENU yaw from a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
