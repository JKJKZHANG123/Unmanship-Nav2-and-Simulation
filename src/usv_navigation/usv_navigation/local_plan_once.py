"""Request one Nav2 path in the Point-LIO local frame and report its contents."""

import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as PathMsg
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


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


def _yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _path_length(path):
    return sum(
        math.hypot(
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y,
        )
        for previous, current in zip(path.poses, path.poses[1:])
    )


class LocalPlanOnce(Node):
    """Call ComputePathToPose without starting any motion-producing nodes."""

    def __init__(self):
        super().__init__('local_plan_once')
        self.declare_parameter('goal_x', 15.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('frame_id', 'camera_init')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('action_name', '/compute_path_to_pose')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('server_timeout_sec', 10.0)
        self.declare_parameter('result_timeout_sec', 15.0)
        self.declare_parameter('hold_seconds', 15.0)
        self.declare_parameter('output_file', '/tmp/usv_local_plan.json')

        self._goal_x = self.get_parameter('goal_x').value
        self._goal_y = self.get_parameter('goal_y').value
        self._goal_yaw = self.get_parameter('goal_yaw').value
        self._frame_id = self.get_parameter('frame_id').value
        self._planner_id = self.get_parameter('planner_id').value
        self._action_name = self.get_parameter('action_name').value
        self._server_timeout = self.get_parameter('server_timeout_sec').value
        self._result_timeout = self.get_parameter('result_timeout_sec').value
        self._hold_seconds = self.get_parameter('hold_seconds').value
        self._output_file = self.get_parameter('output_file').value

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._path_publisher = self.create_publisher(
            PathMsg, self.get_parameter('path_topic').value, qos)
        self._client = ActionClient(
            self, ComputePathToPose, self._action_name)

    def execute(self):
        if not all(math.isfinite(value) for value in (
                self._goal_x, self._goal_y, self._goal_yaw)):
            self.get_logger().error('Goal contains a non-finite value')
            return 2

        self.get_logger().info(
            f'Waiting for {self._action_name} (timeout '
            f'{self._server_timeout:.1f}s)')
        if not self._client.wait_for_server(timeout_sec=self._server_timeout):
            self.get_logger().error(
                f'Action server {self._action_name} is unavailable')
            return 3

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = self._frame_id
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(self._goal_x)
        goal.goal.pose.position.y = float(self._goal_y)
        goal.goal.pose.orientation.z = math.sin(self._goal_yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(self._goal_yaw / 2.0)
        goal.planner_id = self._planner_id
        # Nav2 obtains the current base_link pose from camera_init -> base_link.
        goal.use_start = False

        self.get_logger().info(
            f'Requesting local path: frame={self._frame_id}, '
            f'goal=({self._goal_x:.3f}, {self._goal_y:.3f}), '
            f'yaw={self._goal_yaw:.3f} rad, planner={self._planner_id}')
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=self._server_timeout)
        if not send_future.done():
            self.get_logger().error('Timed out waiting for goal acknowledgement')
            return 4

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Planner rejected the goal')
            return 5

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._result_timeout)
        if not result_future.done():
            self.get_logger().error('Timed out waiting for planner result')
            goal_handle.cancel_goal_async()
            return 6

        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error('Planner returned no result')
            return 7
        result = wrapped_result.result
        error_name = _ERROR_NAMES.get(
            result.error_code, f'UNRECOGNIZED_{result.error_code}')
        if (wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or
                result.error_code != ComputePathToPose.Result.NONE):
            self.get_logger().error(
                f'Planning failed: action_status={wrapped_result.status}, '
                f'error={error_name}({result.error_code}), '
                f'message={result.error_msg!r}')
            return 8
        if not result.path.poses:
            self.get_logger().error('Planner reported success but path is empty')
            return 9

        length = _path_length(result.path)
        start = result.path.poses[0].pose.position
        end = result.path.poses[-1].pose.position
        goal_error = math.hypot(end.x - self._goal_x, end.y - self._goal_y)
        planning_time = (
            result.planning_time.sec + result.planning_time.nanosec * 1e-9)

        # Publish the action result on the repository's standard path topic.
        self._path_publisher.publish(result.path)
        self.get_logger().info('===== LOCAL PLANNING SUCCESS =====')
        self.get_logger().info(
            f'path_type=nav_msgs/msg/Path frame={result.path.header.frame_id!r}')
        self.get_logger().info(
            f'poses={len(result.path.poses)} length={length:.3f} m '
            f'planning_time={planning_time:.6f} s')
        self.get_logger().info(
            f'start=({start.x:.3f}, {start.y:.3f}, {start.z:.3f}) '
            f'end=({end.x:.3f}, {end.y:.3f}, {end.z:.3f}) '
            f'goal_error={goal_error:.3f} m')
        self.get_logger().info(
            f'error={error_name}({result.error_code}) '
            f'message={result.error_msg!r}')

        if self._output_file:
            self._write_json(result.path, planning_time, length, goal_error)

        if self._hold_seconds > 0.0:
            self.get_logger().info(
                f'Publishing /plan for {self._hold_seconds:.1f}s; inspect it '
                'with: ros2 topic echo /plan --once')
            deadline = time.monotonic() + self._hold_seconds
            while rclpy.ok() and time.monotonic() < deadline:
                self._path_publisher.publish(result.path)
                rclpy.spin_once(self, timeout_sec=0.5)
        return 0

    def _write_json(self, path, planning_time, length, goal_error):
        document = {
            'message_type': 'nav_msgs/msg/Path',
            'frame_id': path.header.frame_id,
            'stamp': {
                'sec': path.header.stamp.sec,
                'nanosec': path.header.stamp.nanosec,
            },
            'goal': {
                'x': self._goal_x,
                'y': self._goal_y,
                'yaw': self._goal_yaw,
            },
            'planner_id': self._planner_id,
            'planning_time_sec': planning_time,
            'path_length_m': length,
            'goal_error_m': goal_error,
            'pose_count': len(path.poses),
            'poses': [],
        }
        for index, stamped_pose in enumerate(path.poses):
            position = stamped_pose.pose.position
            orientation = stamped_pose.pose.orientation
            document['poses'].append({
                'index': index,
                'x': position.x,
                'y': position.y,
                'z': position.z,
                'yaw': _yaw_from_quaternion(orientation),
                'orientation': {
                    'x': orientation.x,
                    'y': orientation.y,
                    'z': orientation.z,
                    'w': orientation.w,
                },
            })
        try:
            output_path = Path(self._output_file).expanduser()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_text(
                json.dumps(document, indent=2, ensure_ascii=False) + '\n',
                encoding='utf-8')
            self.get_logger().info(f'Path JSON written to {output_path}')
        except OSError as error:
            self.get_logger().error(
                f'Could not write path JSON {self._output_file!r}: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanOnce()
    exit_code = 1
    try:
        exit_code = node.execute()
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
