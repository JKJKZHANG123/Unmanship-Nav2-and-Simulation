#!/usr/bin/env python3
"""Convert a sequence of WGS84 waypoints into a metric ``PoseArray``.

This is the multi-waypoint analogue of ``target_to_goal``: it reads an ordered
list of lat/lon/alt points (``geographic_msgs/GeoPath``), validates the whole
route as one unit, transforms each point to the ``camera_init`` frame, and
publishes a single ``geometry_msgs/PoseArray`` on ``/goal_poses``.
``geodetic_goal_planner`` then submits the whole list to Nav2's
``ComputePathThroughPoses`` for one continuous path through every point.

Validation is deliberately strict: a single invalid waypoint rejects the whole
route.  On a real boat, silently dropping a waypoint and planning the remaining
subset would let the operator believe N points were planned while the hull only
visits a fraction of them.
"""

import math
from typing import List, Optional, Tuple

from geographic_msgs.msg import GeoPath
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener

from .geodesy import lat_lon_to_utm, validate_lat_lon, yaw_to_quaternion


class TargetWaypointsToGoals(Node):
    def __init__(self):
        super().__init__('target_waypoints_to_goals')
        self.declare_parameter('waypoints_topic', '/target/geopath')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('goals_topic', '/goal_poses')
        self.declare_parameter('global_frame', 'camera_init')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('goal_altitude_m', 0.0)
        self.declare_parameter('tf_timeout_s', 0.5)
        self.declare_parameter('max_waypoints', 200)
        self.declare_parameter('strict_waypoint_validation', True)
        self.declare_parameter('reject_zone_mismatch', True)
        self.declare_parameter('max_waypoint_distance_m', 5000.0)
        self.declare_parameter('max_segment_distance_m', 0.0)

        self._global_frame = str(self.get_parameter('global_frame').value)
        self._utm_frame = str(self.get_parameter('utm_frame').value)
        self._goal_altitude = float(self.get_parameter('goal_altitude_m').value)
        self._tf_timeout = float(self.get_parameter('tf_timeout_s').value)
        self._max_waypoints = int(self.get_parameter('max_waypoints').value)
        self._strict = bool(self.get_parameter('strict_waypoint_validation').value)
        self._reject_zone = bool(self.get_parameter('reject_zone_mismatch').value)
        self._max_waypoint_dist = float(
            self.get_parameter('max_waypoint_distance_m').value)
        self._max_segment_dist = float(
            self.get_parameter('max_segment_distance_m').value)

        self._boat: Optional[NavSatFix] = None

        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._goals_pub = self.create_publisher(
            PoseArray, str(self.get_parameter('goals_topic').value), q)
        self.create_subscription(
            GeoPath, str(self.get_parameter('waypoints_topic').value),
            self._on_waypoints, q)
        self.create_subscription(
            NavSatFix, str(self.get_parameter('gps_topic').value),
            self._on_boat, q)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.get_logger().info(
            f'{self.get_parameter("waypoints_topic").value} -> '
            f'{self.get_parameter("goals_topic").value} '
            f'({self._global_frame}); strict={self._strict}, '
            f'reject_zone={self._reject_zone}')

    def _on_boat(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX and validate_lat_lon(
                msg.latitude, msg.longitude):
            self._boat = msg

    def _on_waypoints(self, msg: GeoPath):
        if not msg.poses:
            self.get_logger().warn('ignored empty /target/geopath')
            return
        if len(msg.poses) > self._max_waypoints:
            self.get_logger().warn(
                f'clamping {len(msg.poses)} waypoints to {self._max_waypoints}')
            msg.poses = msg.poses[:self._max_waypoints]

        # ---- pass 1: validate every point and convert to UTM as one unit ----
        utm_points: List[Tuple[float, float]] = []
        reference_zone = reference_north = None
        for index, geo_pose in enumerate(msg.poses):
            lat = float(geo_pose.pose.position.latitude)
            lon = float(geo_pose.pose.position.longitude)
            alt = float(geo_pose.pose.position.altitude)
            if not validate_lat_lon(lat, lon) or not math.isfinite(alt):
                if self._strict:
                    self.get_logger().error(
                        f'rejected route: waypoint {index} has invalid '
                        f'lat/lon/alt')
                    return
                self.get_logger().warn(
                    f'skipping invalid waypoint {index} '
                    f'(strict_waypoint_validation=false)')
                continue
            east, north, zone, northern = lat_lon_to_utm(lat, lon)
            if reference_zone is None:
                reference_zone = zone
                reference_north = northern
            elif (self._reject_zone and
                  (zone != reference_zone or northern != reference_north)):
                self.get_logger().error(
                    f'rejected route: waypoint {index} UTM zone {zone} '
                    f'({northern}) differs from route zone {reference_zone} '
                    f'({reference_north})')
                return
            utm_points.append((east, north))

        if not utm_points:
            self.get_logger().warn('no valid waypoints after filtering')
            return

        # ---- pass 2: distance limits relative to the boat and to neighbours ----
        if self._boat is not None:
            boat_easting, boat_northing, boat_zone, boat_north = lat_lon_to_utm(
                self._boat.latitude, self._boat.longitude)
            if (self._reject_zone and reference_zone is not None and
                    (boat_zone != reference_zone or boat_north != reference_north)):
                self.get_logger().error(
                    f'rejected route: boat UTM zone {boat_zone} differs from '
                    f'route zone {reference_zone}')
                return
            if self._max_waypoint_dist > 0.0:
                for index, (east, north) in enumerate(utm_points):
                    distance = math.hypot(east - boat_easting,
                                          north - boat_northing)
                    if distance > self._max_waypoint_dist:
                        self.get_logger().error(
                            f'rejected route: waypoint {index} is '
                            f'{distance:.1f}m from the boat, exceeds '
                            f'{self._max_waypoint_dist:.1f}m')
                        return

        if self._max_segment_dist > 0.0:
            for index in range(1, len(utm_points)):
                prev = utm_points[index - 1]
                curr = utm_points[index]
                distance = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
                if distance > self._max_segment_dist:
                    self.get_logger().error(
                        f'rejected route: segment {index - 1}->{index} is '
                        f'{distance:.1f}m, exceeds {self._max_segment_dist:.1f}m')
                    return

        # ---- pass 3: TF lookup, then build goals with yaw from the NEXT point --
        try:
            transform = self._tf.lookup_transform(
                self._global_frame, self._utm_frame, rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout))
        except TransformException as exc:
            self.get_logger().warn(
                f'cannot transform waypoints to {self._global_frame}: {exc}')
            return

        goals = PoseArray()
        goals.header.stamp = self.get_clock().now().to_msg()
        goals.header.frame_id = self._global_frame

        for index, (east, north) in enumerate(utm_points):
            if index + 1 < len(utm_points):
                nxt_east, nxt_north = utm_points[index + 1]
                yaw = math.atan2(nxt_north - north, nxt_east - east)
            else:
                yaw = 0.0

            pose_utm = PoseStamped()
            pose_utm.header.stamp = goals.header.stamp
            pose_utm.header.frame_id = self._utm_frame
            pose_utm.pose.position.x = east
            pose_utm.pose.position.y = north
            pose_utm.pose.position.z = self._goal_altitude
            pose_utm.pose.orientation = yaw_to_quaternion(yaw)

            goal = do_transform_pose_stamped(pose_utm, transform)
            pose = Pose()
            pose.position.x = float(goal.pose.position.x)
            pose.position.y = float(goal.pose.position.y)
            pose.position.z = float(goal.pose.position.z)
            pose.orientation = goal.pose.orientation
            goals.poses.append(pose)

        self._goals_pub.publish(goals)
        self.get_logger().info(
            f'published {len(goals.poses)} waypoint goal(s) on /goal_poses '
            f'in {self._global_frame}')


def main(args=None):
    rclpy.init(args=args)
    node = TargetWaypointsToGoals()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
