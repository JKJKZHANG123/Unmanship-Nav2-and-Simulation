"""Plan to RViz clicked points without starting a Nav2 controller."""

import json
import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path as PathMsg
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


_ERROR_NAMES = {
    ComputePathToPose.Result.NONE: 'NONE',
    ComputePathToPose.Result.UNKNOWN: 'UNKNOWN',
    ComputePathToPose.Result.INVALID_PLANNER: 'INVALID_PLANNER',
    ComputePathToPose.Result.TF_ERROR: 'TF_ERROR',
    ComputePathToPose.Result.START_OUTSIDE_MAP: 'START_OUTSIDE_MAP',
    ComputePathToPose.Result.GOAL_OUTSIDE_MAP: 'GOAL_OUTSIDE_MAP',
    ComputePathToPose.Result.START_OCCUPIED: 'START_OCCUPIED',
    ComputePathToPose.Result.GOAL_OCCUPIED: 'GOAL_OCCUPIED',
    ComputePathToPose.Result.TIMEOUT: 'TIMEOUT',
    ComputePathToPose.Result.NO_VALID_PATH: 'NO_VALID_PATH',
}


def _yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z +
               quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y +
                     quaternion.z * quaternion.z))


def _length(path):
    return sum(
        math.hypot(
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y)
        for previous, current in zip(path.poses, path.poses[1:]))


class LocalPlanClick(Node):
    """Convert RViz /clicked_point messages into ComputePathToPose calls."""

    def __init__(self):
        super().__init__('local_plan_click')
        self.declare_parameter('frame_id', 'camera_init')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('odom_topic', '/aft_mapped_to_init')
        self.declare_parameter('action_name', '/compute_path_to_pose')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('goal_marker_topic', '/nav_waypoints_markers')
        self.declare_parameter('goal_yaw_mode', 'bearing')
        self.declare_parameter('fixed_goal_yaw', 0.0)
        self.declare_parameter('output_file', '/tmp/usv_local_plan.json')

        self._frame_id = self.get_parameter('frame_id').value
        self._planner_id = self.get_parameter('planner_id').value
        self._yaw_mode = self.get_parameter('goal_yaw_mode').value
        self._fixed_yaw = self.get_parameter('fixed_goal_yaw').value
        self._output_file = self.get_parameter('output_file').value
        self._pose = None
        self._busy = False
        self._requested_goal = None

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._path_publisher = self.create_publisher(
            PathMsg, self.get_parameter('path_topic').value, path_qos)
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter('goal_marker_topic').value,
            path_qos)
        self.create_subscription(
            PointStamped,
            self.get_parameter('clicked_point_topic').value,
            self._on_click,
            10)
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._on_odom,
            20)
        self._client = ActionClient(
            self,
            ComputePathToPose,
            self.get_parameter('action_name').value)

        self.get_logger().info(
            'Planner-only click node ready: select RViz Publish Point and '
            f'click in frame {self._frame_id}; no motion command is generated')

    def _on_odom(self, message):
        self._pose = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            _yaw(message.pose.pose.orientation),
        )

    def _on_click(self, message):
        frame = message.header.frame_id or self._frame_id
        if frame != self._frame_id:
            self.get_logger().error(
                f'Rejected clicked point in frame {frame!r}; expected '
                f'{self._frame_id!r}')
            return
        if self._pose is None:
            self.get_logger().error(
                'Rejected clicked point: no /aft_mapped_to_init odometry yet')
            return
        if self._busy:
            self.get_logger().warning(
                'A planning request is already running; click again after it '
                'finishes')
            return
        if not self._client.server_is_ready():
            self.get_logger().error(
                '/compute_path_to_pose is not ready; check planner_server is '
                'active')
            return

        x = float(message.point.x)
        y = float(message.point.y)
        if not math.isfinite(x) or not math.isfinite(y):
            self.get_logger().error('Rejected non-finite clicked coordinates')
            return

        if self._yaw_mode == 'bearing':
            goal_yaw = math.atan2(y - self._pose[1], x - self._pose[0])
        elif self._yaw_mode == 'current':
            goal_yaw = self._pose[2]
        elif self._yaw_mode == 'fixed':
            goal_yaw = self._fixed_yaw
        else:
            self.get_logger().error(
                f'Unknown goal_yaw_mode {self._yaw_mode!r}; use bearing, '
                'current, or fixed')
            return

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = self._frame_id
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(goal_yaw / 2.0)
        goal.planner_id = self._planner_id
        # The planner gets the current vessel pose and heading from Point-LIO TF.
        goal.use_start = False

        self._busy = True
        self._requested_goal = (x, y, goal_yaw)
        self._publish_goal_marker(x, y)
        self.get_logger().info(
            f'Clicked goal ({x:.3f}, {y:.3f}), yaw={goal_yaw:.3f} rad; '
            'requesting path only')
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: B902 - ROS future propagates errors
            self.get_logger().error(f'Goal request failed: {error}')
            self._busy = False
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Planner rejected clicked goal')
            self._busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            wrapped_result = future.result()
        except Exception as error:  # noqa: B902 - ROS future propagates errors
            self.get_logger().error(f'Planning result failed: {error}')
            self._busy = False
            return

        result = wrapped_result.result
        error_name = _ERROR_NAMES.get(
            result.error_code, f'UNRECOGNIZED_{result.error_code}')
        if (wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or
                result.error_code != ComputePathToPose.Result.NONE):
            self.get_logger().error(
                f'Planning failed: status={wrapped_result.status}, '
                f'error={error_name}({result.error_code}), '
                f'message={result.error_msg!r}')
            self._busy = False
            return
        if not result.path.poses:
            self.get_logger().error('Planner returned an empty path')
            self._busy = False
            return

        length = _length(result.path)
        start = result.path.poses[0].pose.position
        end = result.path.poses[-1].pose.position
        goal_x, goal_y, goal_yaw = self._requested_goal
        goal_error = math.hypot(end.x - goal_x, end.y - goal_y)
        planning_time = (
            result.planning_time.sec + result.planning_time.nanosec * 1e-9)

        self._path_publisher.publish(result.path)
        self.get_logger().info('===== CLICKED LOCAL PLAN SUCCESS =====')
        self.get_logger().info(
            f'frame={result.path.header.frame_id!r} '
            f'poses={len(result.path.poses)} length={length:.3f}m '
            f'planning_time={planning_time:.6f}s')
        self.get_logger().info(
            f'start=({start.x:.3f}, {start.y:.3f}) '
            f'end=({end.x:.3f}, {end.y:.3f}) '
            f'goal_error={goal_error:.3f}m '
            f'error={error_name}({result.error_code})')
        self._write_json(
            result.path, goal_x, goal_y, goal_yaw,
            length, planning_time, goal_error)
        self._busy = False

    def _publish_goal_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'local_planning_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 1.0
        self._marker_publisher.publish(MarkerArray(markers=[marker]))

    def _write_json(
            self, path, goal_x, goal_y, goal_yaw,
            length, planning_time, goal_error):
        if not self._output_file:
            return
        document = {
            'message_type': 'nav_msgs/msg/Path',
            'frame_id': path.header.frame_id,
            'planner_id': self._planner_id,
            'goal': {'x': goal_x, 'y': goal_y, 'yaw': goal_yaw},
            'pose_count': len(path.poses),
            'path_length_m': length,
            'planning_time_sec': planning_time,
            'goal_error_m': goal_error,
            'poses': [{
                'index': index,
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z,
                'yaw': _yaw(pose.pose.orientation),
                'orientation': {
                    'x': pose.pose.orientation.x,
                    'y': pose.pose.orientation.y,
                    'z': pose.pose.orientation.z,
                    'w': pose.pose.orientation.w,
                },
            } for index, pose in enumerate(path.poses)],
        }
        try:
            output = Path(self._output_file).expanduser()
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(document, ensure_ascii=False, indent=2) + '\n',
                encoding='utf-8')
            self.get_logger().info(f'Path JSON written to {output}')
        except OSError as error:
            self.get_logger().error(f'Could not write path JSON: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanClick()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
