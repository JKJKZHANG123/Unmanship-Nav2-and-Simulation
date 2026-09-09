#!/usr/bin/env python3
"""Send an ordered waypoint sequence for path-only multi-goal planning.

Two input forms are supported:

* Metric ``camera_init`` waypoints (default)::

    ./send_waypoints.py 10 0 20 5 30 0            # (x1,y1),(x2,y2),(x3,y3)

* WGS84 lat/lon waypoints::

    ./send_waypoints.py --geodetic 31.2304 121.4737 31.2310 121.4740

Metric coordinates are published directly on ``/goal_poses`` for
``geodetic_goal_planner``.  Geodetic coordinates are published on
``/target/geopath`` (``geographic_msgs/GeoPath``) for
``target_waypoints_to_goals``, which transforms them to ``/goal_poses``.

The script only publishes a plan request; it never sends ``NavigateToPose`` or
a velocity command.  Use ``ros2 topic pub /cancel_navigation std_msgs/Empty``
to clear the active route.
"""

import argparse
import math
import sys

from geographic_msgs.msg import GeoPath, GeoPoseStamped
from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def _quaternion(yaw):
    from geometry_msgs.msg import Quaternion
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0),
                      w=math.cos(yaw / 2.0))


class WaypointSender(Node):
    """Publish an ordered waypoint route on /goal_poses or /target/geopath."""

    def __init__(self, geodetic: bool):
        """Create the sender and the matching publisher."""
        super().__init__('send_waypoints')
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._geodetic = geodetic
        if geodetic:
            self._geo_pub = self.create_publisher(
                GeoPath, '/target/geopath', q)
        else:
            self._goals_pub = self.create_publisher(
                PoseArray, '/goal_poses', q)

    def send_metric(self, xy):
        """Publish camera_init (x, y) pairs as a /goal_poses PoseArray."""
        goals = PoseArray()
        goals.header.stamp = self.get_clock().now().to_msg()
        goals.header.frame_id = 'camera_init'
        for x, y in xy:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0
            pose.orientation = _quaternion(0.0)
            goals.poses.append(pose)
        self._goals_pub.publish(goals)
        self.get_logger().info(
            f'sent {len(xy)} metric waypoint(s) on /goal_poses')

    def send_geodetic(self, latlon):
        """Publish WGS84 (lat, lon) pairs as a /target/geopath GeoPath."""
        geo = GeoPath()
        geo.header.stamp = self.get_clock().now().to_msg()
        geo.header.frame_id = 'wgs84'
        for lat, lon in latlon:
            geo_pose = GeoPoseStamped()
            geo_pose.header = geo.header
            geo_pose.pose.position.latitude = float(lat)
            geo_pose.pose.position.longitude = float(lon)
            geo_pose.pose.position.altitude = 0.0
            geo_pose.pose.orientation = _quaternion(0.0)
            geo.poses.append(geo_pose)
        self._geo_pub.publish(geo)
        self.get_logger().info(
            f'sent {len(latlon)} geodetic waypoint(s) on /target/geopath')


def main() -> int:
    """Parse CLI args and publish one waypoint route, then exit."""
    parser = argparse.ArgumentParser(
        description='Send an ordered waypoint route')
    parser.add_argument('values', nargs='+', type=float,
                        help='x y [x y ...] (metric) or lat lon [lat lon ...]')
    parser.add_argument('--geodetic', action='store_true',
                        help='interpret values as WGS84 lat/lon pairs')
    parser.add_argument('--once', action='store_true',
                        help='publish once and exit (default repeats briefly)')
    args = parser.parse_args()

    if len(args.values) % 2 != 0:
        print('error: expected an even number of values (x y pairs)',
              file=sys.stderr)
        return 2
    if len(args.values) < 4:
        print('error: need at least two waypoints (4 values)',
              file=sys.stderr)
        return 2
    pairs = list(zip(args.values[0::2], args.values[1::2]))

    rclpy.init()
    node = WaypointSender(args.geodetic)
    try:
        if args.geodetic:
            node.send_geodetic(pairs)
        else:
            node.send_metric(pairs)
        if not args.once:
            # Keep spinning so a transient-local subscriber can receive it.
            import time
            deadline = time.monotonic() + 0.5
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
