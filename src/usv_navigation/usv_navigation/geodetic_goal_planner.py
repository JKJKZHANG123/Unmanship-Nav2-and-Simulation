#!/usr/bin/env python3
"""Compute a single obstacle-aware path through a sequence of metric goals.

This node is intentionally path-only: it uses Nav2 ``ComputePathThroughPoses``
and never sends ``NavigateToPose`` or velocity commands.  The whole ordered
waypoint list is handed to the planner in one action, so SmacPlannerHybrid
returns one continuous path that visits every waypoint in order -- not a
concatenation of independent per-segment paths with possible seams or detours.

A new route cancels the old planning request.  Any terminal planning error
clears the published path and the active route so a stale path cannot be
mistaken for a valid command.
"""

import copy
import math
import time
from typing import List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import ComputePathThroughPoses
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String


_ERROR_NAMES = {
    ComputePathThroughPoses.Result.NONE: 'NONE',
    ComputePathThroughPoses.Result.UNKNOWN: 'UNKNOWN',
    ComputePathThroughPoses.Result.INVALID_PLANNER: 'INVALID_PLANNER',
    ComputePathThroughPoses.Result.TF_ERROR: 'TF_ERROR',
    ComputePathThroughPoses.Result.START_OUTSIDE_MAP: 'START_OUTSIDE_MAP',
    ComputePathThroughPoses.Result.GOAL_OUTSIDE_MAP: 'GOAL_OUTSIDE_MAP',
    ComputePathThroughPoses.Result.START_OCCUPIED: 'START_OCCUPIED',
    ComputePathThroughPoses.Result.GOAL_OCCUPIED: 'GOAL_OCCUPIED',
    ComputePathThroughPoses.Result.TIMEOUT: 'TIMEOUT',
    ComputePathThroughPoses.Result.NO_VALID_PATH: 'NO_VALID_PATH',
    ComputePathThroughPoses.Result.NO_VIAPOINTS_GIVEN: 'NO_VIAPOINTS_GIVEN',
}


def _path_length(path: Path) -> float:
    return sum(math.hypot(
        current.pose.position.x - previous.pose.position.x,
        current.pose.position.y - previous.pose.position.y)
        for previous, current in zip(path.poses, path.poses[1:]))


def _finite_pose(pose: PoseStamped) -> bool:
    values = (
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        pose.pose.orientation.x, pose.pose.orientation.y,
        pose.pose.orientation.z, pose.pose.orientation.w)
    return bool(pose.header.frame_id) and all(math.isfinite(float(v)) for v in values)


class GeodeticGoalPlanner(Node):
    """Build and publish a safe concatenated path for a route."""

    def __init__(self):
        super().__init__('geodetic_goal_planner')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('goal_poses_topic', '/goal_poses')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('cancel_topic', '/cancel_navigation')
        self.declare_parameter('status_topic', '/usv/goal_status')
        self.declare_parameter('action_name', '/compute_path_through_poses')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('server_timeout_s', 2.0)
        self.declare_parameter('retry_period_s', 1.0)
        self.declare_parameter('replan_period_s', 1.0)
        self.declare_parameter('max_retries', 0)
        self.declare_parameter('abort_on_planning_error', True)

        self._goal_topic = str(self.get_parameter('goal_topic').value)
        self._goal_poses_topic = str(self.get_parameter('goal_poses_topic').value)
        self._path_topic = str(self.get_parameter('path_topic').value)
        self._action_name = str(self.get_parameter('action_name').value)
        self._planner_id = str(self.get_parameter('planner_id').value)
        self._server_timeout = max(0.1, float(self.get_parameter('server_timeout_s').value))
        self._retry_period = max(0.1, float(self.get_parameter('retry_period_s').value))
        self._replan_period = max(0.1, float(self.get_parameter('replan_period_s').value))
        self._max_retries = max(0, int(self.get_parameter('max_retries').value))
        self._abort_on_error = bool(self.get_parameter('abort_on_planning_error').value)

        path_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        goal_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._path_pub = self.create_publisher(Path, self._path_topic, path_qos)
        self._status_pub = self.create_publisher(String, str(
            self.get_parameter('status_topic').value), path_qos)
        self.create_subscription(PoseStamped, self._goal_topic, self._on_goal, goal_qos)
        self.create_subscription(PoseArray, self._goal_poses_topic,
                                 self._on_goal_poses, goal_qos)
        self.create_subscription(Empty, str(self.get_parameter('cancel_topic').value),
                                 self._on_cancel, goal_qos)
        self._client = ActionClient(self, ComputePathThroughPoses, self._action_name)

        self._route: List[PoseStamped] = []
        self._route_key = None
        self._generation = 0
        self._route_planned = False
        self._busy = False
        self._goal_handle = None
        self._retry_at = 0.0
        self._attempts = 0
        self._action_sent_at = 0.0
        self._last_warn = 0.0
        self._status_sequence = 0
        self.create_timer(0.1, self._tick)
        self._publish_status(False, 'IDLE code=0')
        self.get_logger().info(
            f'{self._goal_topic}/{self._goal_poses_topic} -> {self._action_name} '
            f'-> {self._path_topic}; path-only multi-goal mode')

    @staticmethod
    def _key(route: List[PoseStamped]):
        return tuple(tuple(round(float(v), 6) for v in (
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w))
            for pose in route)

    def _accept_route(self, route: List[PoseStamped], source: str):
        if not route or not all(_finite_pose(pose) for pose in route):
            self.get_logger().error(f'rejected invalid {source} route')
            self._abort_route('invalid route')
            return
        key = self._key(route)
        if key == self._route_key and self._route:
            return
        self._generation += 1
        self._route = copy.deepcopy(route)
        self._route_key = key
        self._route_planned = False
        self._attempts = 0
        self._retry_at = 0.0
        self._cancel_active_goal()
        self._publish_empty_path()
        self._publish_status(False, f'QUEUED code=1; points={len(route)}')
        self.get_logger().info(
            f'accepted {source} route with {len(route)} goal point(s)')

    def _on_goal(self, msg: PoseStamped):
        self._accept_route([msg], 'single-goal')

    def _on_goal_poses(self, msg: PoseArray):
        if not msg.poses:
            self._abort_route('empty route')
            return
        route = []
        for pose in msg.poses:
            stamped = PoseStamped()
            stamped.header = copy.deepcopy(msg.header)
            stamped.pose = copy.deepcopy(pose)
            route.append(stamped)
        self._accept_route(route, 'multi-goal')

    def _on_cancel(self, _msg: Empty):
        self._generation += 1
        self._route = []
        self._route_key = None
        self._route_planned = False
        self._attempts = 0
        self._retry_at = 0.0
        self._cancel_active_goal()
        self._publish_empty_path()
        self._publish_status(True, 'CANCELED code=5; route cleared')
        self.get_logger().warn('active goal/route cancelled and /plan cleared')

    def _abort_route(self, reason: str):
        self._generation += 1
        self._cancel_active_goal()
        self._route = []
        self._route_key = None
        self._route_planned = False
        self._attempts = 0
        self._retry_at = 0.0
        self._publish_empty_path()
        self._publish_status(False, f'ABORTED code=6; {reason}')
        self.get_logger().error(f'route aborted: {reason}')

    def _cancel_active_goal(self):
        handle = self._goal_handle
        self._goal_handle = None
        self._busy = False
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f'failed to cancel ComputePathThroughPoses: {exc}')

    def _publish_empty_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._route:
            msg.header.frame_id = self._route[0].header.frame_id
        self._path_pub.publish(msg)

    def _publish_status(self, accepted: bool, text: str):
        self._status_sequence += 1
        self._status_pub.publish(String(data=(
            f'{text}; accepted={int(accepted)}; '
            f'status_seq={self._status_sequence}')))

    def _warn_throttled(self, text: str):
        now = time.monotonic()
        if now - self._last_warn >= 5.0:
            self.get_logger().warn(text)
            self._last_warn = now

    def _tick(self):
        if not self._route or time.monotonic() < self._retry_at:
            return
        if self._busy:
            # An in-flight action that never returns.  Honour the configured
            # timeout: abort (default) or just re-arm the retry timer.
            if time.monotonic() - self._action_sent_at <= self._server_timeout:
                return
            self.get_logger().warn(
                f'ComputePathThroughPoses result timeout after '
                f'{self._server_timeout:.1f}s')
            self._busy = False
            handle = self._goal_handle
            self._goal_handle = None
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            if self._abort_on_error:
                self._abort_route(
                    f'planner result timeout ({self._server_timeout:.1f}s)')
            else:
                self._retry_at = time.monotonic() + self._retry_period
            return
        if self._route_planned:
            self._route_planned = False
        if self._max_retries > 0 and self._attempts >= self._max_retries:
            self._abort_route(f'planner retry limit reached ({self._max_retries})')
            return
        if not self._client.server_is_ready():
            self._warn_throttled(f'waiting for Nav2 action server {self._action_name}')
            self._retry_at = time.monotonic() + self._retry_period
            return

        goal = ComputePathThroughPoses.Goal()
        # Hand the whole ordered route to the planner as a single request so it
        # returns one continuous path through every waypoint in order.
        goal.goals = copy.deepcopy(self._route)
        goal.planner_id = self._planner_id
        goal.use_start = False
        generation = self._generation
        self._busy = True
        self._attempts += 1
        self._action_sent_at = time.monotonic()
        try:
            future = self._client.send_goal_async(goal)
            future.add_done_callback(
                lambda result_future: self._on_goal_response(
                    result_future, generation))
        except Exception as exc:
            self._busy = False
            self._retry_at = time.monotonic() + self._retry_period
            self.get_logger().error(f'ComputePathThroughPoses send failed: {exc}')

    def _on_goal_response(self, future, generation: int):
        try:
            handle = future.result()
        except Exception as exc:
            if generation != self._generation:
                return
            self._busy = False
            self._retry_at = time.monotonic() + self._retry_period
            self.get_logger().error(f'ComputePathThroughPoses response failed: {exc}')
            return
        if generation != self._generation or not self._route:
            try:
                if handle.accepted:
                    handle.cancel_goal_async()
            except Exception:
                pass
            return
        if not handle.accepted:
            self._busy = False
            self._retry_at = time.monotonic() + self._retry_period
            self._warn_throttled('Nav2 rejected ComputePathThroughPoses request')
            return
        self._goal_handle = handle
        try:
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda result: self._on_result(result, generation, handle))
        except Exception as exc:
            self._busy = False
            self._abort_route(f'cannot monitor planner result: {exc}')

    def _on_result(self, future, generation: int, handle):
        try:
            wrapped = future.result()
        except Exception as exc:
            wrapped = None
            error = str(exc)
        else:
            error = ''
        if generation != self._generation or not self._route:
            # A superseded action's late result must not touch the current
            # _busy/_goal_handle state, or it would clear the newer route's
            # in-flight flag and make _tick send a duplicate action.
            return
        if self._goal_handle is handle:
            self._goal_handle = None
        self._busy = False
        if wrapped is None:
            self._abort_route(f'planner result failed: {error}')
            return

        result = wrapped.result
        error_name = _ERROR_NAMES.get(
            result.error_code, f'UNRECOGNIZED_{result.error_code}')
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                result.error_code != ComputePathThroughPoses.Result.NONE or
                not result.path.poses):
            self._abort_route(
                f'planning failed points={len(self._route)} '
                f'status={wrapped.status} error={error_name} '
                f'message={result.error_msg!r}')
            return

        self._path_pub.publish(result.path)
        self._route_planned = True
        self._attempts = 0
        self._retry_at = time.monotonic() + self._replan_period
        self._publish_status(True, f'PLANNED code=3; points={len(result.path.poses)}')
        self.get_logger().info(
            f'published path-through route: {len(result.path.poses)} poses, '
            f'{_path_length(result.path):.2f} m, {len(self._route)} goal point(s)')


def main(args=None):
    rclpy.init(args=args)
    node = GeodeticGoalPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._cancel_active_goal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
