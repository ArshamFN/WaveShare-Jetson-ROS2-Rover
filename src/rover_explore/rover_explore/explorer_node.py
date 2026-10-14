"""Rover explorer node: frontier exploration with the rover's own planner.

Replaces explore_lite. The decisions come from frontier_planner (the same code
the offline simulator in tools/explore_sim runs); this node only connects it
to ROS:

- reads the SLAM map from /map and the robot pose from TF (map -> base_link),
- re-plans every planner_period_s, and right after each Nav2 goal ends (once
  the map has been updated since, or after map_wait_s),
- sends one NavigateToPose goal at a time and re-sends only when the target
  viewpoint moves; a new goal preempts the old one in Nav2, and results of
  replaced goals are ignored,
- publishes the same /explore/status messages as explore_lite and obeys the
  same /explore/resume topic (false pauses and cancels this node's goal, true
  resumes), so explore_autosave works unchanged,
- declares exploration complete only after done_confirmations decisions in a
  row, each on a newer map, find no reachable frontier; it then cancels its
  own goal (never anyone else's) and logs one completion line and one stats
  line,
- publishes frontier markers on /explore/frontiers for Foxglove.

The node never publishes velocity commands; Nav2 does all the driving.
"""

import math
import time

from action_msgs.msg import GoalStatus
from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rover_explore.frontier_planner import GridMap, Params, Planner, belief_from_occupancy

NAV_ACTION = '/navigate_to_pose'
STATUS_TOPIC = '/explore/status'
RESUME_TOPIC = '/explore/resume'
MAP_TOPIC = '/map'
MARKER_TOPIC = '/explore/frontiers'

# explore_autosave records this line as the completion cause.
COMPLETION_LINE = 'No reachable frontiers left, stopping.'

TICK_PERIOD_S = 0.25
GOAL_RESPONSE_TIMEOUT_S = 10.0
WAIT_WARN_PERIOD_S = 15.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SentGoal:
    """One NavigateToPose goal and what became of it."""

    def __init__(self, seq, x, y, yaw, now):
        self.seq = seq
        self.x, self.y, self.yaw = x, y, yaw
        self.sent = now
        self.handle = None
        self.cancel_requested = False


class RoverExplorer(Node):

    def __init__(self):
        super().__init__('rover_explorer')
        defaults = Params()
        p = {}
        for name, value in vars(defaults).items():
            p[name] = type(value)(self.declare_parameter(name, value).value)
        self.params = Params(**p)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.base_frame = str(self.declare_parameter('robot_base_frame', 'base_link').value)
        self.planner_period = float(self.declare_parameter('planner_period_s', 2.0).value)
        self.map_wait = float(self.declare_parameter('map_wait_s', 3.0).value)
        self.tf_max_age = float(self.declare_parameter('tf_max_age_s', 1.0).value)
        self.done_confirmations = int(self.declare_parameter('done_confirmations', 2).value)
        self.visualize = bool(self.declare_parameter('visualize', True).value)

        if self.planner_period <= 0.0 or self.map_wait < 0.0 or self.tf_max_age <= 0.0:
            raise ValueError(
                'planner_period_s and tf_max_age_s must be above 0, map_wait_s 0 or more')
        if self.done_confirmations < 1:
            raise ValueError('done_confirmations must be 1 or more')
        if not self.map_frame or not self.base_frame:
            raise ValueError('map_frame and robot_base_frame must not be empty')

        self.planner = Planner(self.params)
        self.state = 'waiting'          # 'waiting', 'exploring', 'paused' or 'complete'
        self.map_msg = None
        self.map_rx = 0.0               # monotonic time the latest map arrived
        self.goal = None                # SentGoal Nav2 is (or should be) working on
        self.goal_seq = 0
        self.pending_event = None       # ('succeeded' | 'failed' | 'canceled', monotonic time)
        self.next_plan = 0.0
        self.last_plan_map_rx = -1.0
        self.done_count = 0
        self.waiting_since = time.monotonic()
        self.last_wait_warn = 0.0
        self.t_started = None
        self.counts = dict(goals_sent=0, replaced=0, succeeded=0, failed=0, rejected=0,
                           canceled_by_others=0)
        self.plan_times = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, NAV_ACTION)

        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(ExploreStatus, STATUS_TOPIC, latched)
        self.marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, 10)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.on_map, map_qos)
        self.create_subscription(Bool, RESUME_TOPIC, self.on_resume, 10)
        self.create_timer(TICK_PERIOD_S, self.tick)

        self.get_logger().info(
            f'Rover explorer ready: {self.map_frame}->{self.base_frame}, '
            f'planner_period_s={self.planner_period:g}, map_wait_s={self.map_wait:g}, '
            f'done_confirmations={self.done_confirmations}, planner {self.params}')

    # ------------------------------------------------------------ inputs
    def on_map(self, msg):
        self.map_msg = msg
        self.map_rx = time.monotonic()

    def on_resume(self, msg):
        if msg.data:
            if self.state == 'paused':
                self.state = 'exploring' if self.t_started is not None else 'waiting'
                self.next_plan = 0.0
                self.publish_status(ExploreStatus.EXPLORATION_IN_PROGRESS)
                self.get_logger().info('Exploration resuming.')
            else:
                self.get_logger().info(f'Resume ignored: exploration is {self.state}.')
            return
        # Like explore_lite, a stop always publishes exploration_paused, even after
        # completion: explore_autosave cancels a return in progress when it sees it.
        self.pending_event = None
        self.cancel_goal('stopped')
        if self.state in ('exploring', 'waiting'):
            self.state = 'paused'
        self.publish_status(ExploreStatus.EXPLORATION_PAUSED)
        self.get_logger().info(f'Exploration stopped ({self.state}).')

    # ------------------------------------------------------------ helpers
    def publish_status(self, status):
        msg = ExploreStatus()
        msg.status = status
        self.status_pub.publish(msg)

    def robot_pose(self):
        """(x, y, yaw) in the map frame, or (None, reason)."""
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException as exc:
            return None, ' '.join(str(exc).split())[:200] or type(exc).__name__
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        if age > self.tf_max_age:
            return None, f'newest {self.map_frame}->{self.base_frame} transform is {age:.1f} s old'
        t = tf.transform
        return (t.translation.x, t.translation.y, yaw_of(t.rotation)), ''

    def grid_map(self):
        m = self.map_msg
        if m is None:
            return None, f'no map on {MAP_TOPIC} yet'
        if m.header.frame_id and m.header.frame_id.lstrip('/') != self.map_frame:
            return None, f'map frame is {m.header.frame_id}, expected {self.map_frame}'
        q = m.info.origin.orientation
        if abs(yaw_of(q)) > 1e-3:
            return None, 'map origin is rotated; not supported'
        if m.info.width == 0 or m.info.height == 0 or len(m.data) != m.info.width * m.info.height:
            return None, 'map is empty or malformed'
        b = belief_from_occupancy(m.data, m.info.width, m.info.height)
        return GridMap(b, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y), ''

    def warn_waiting(self, now, reason):
        if now - self.last_wait_warn >= WAIT_WARN_PERIOD_S:
            self.last_wait_warn = now
            self.get_logger().warn(f'Waiting ({now - self.waiting_since:.0f} s): {reason}')

    # ------------------------------------------------------------ main loop
    def tick(self):
        now = time.monotonic()
        if self.goal is not None and self.goal.handle is None and \
                not self.goal.cancel_requested and now - self.goal.sent > GOAL_RESPONSE_TIMEOUT_S:
            self.get_logger().warn(f'Goal {self.goal.seq}: Nav2 did not answer within '
                                   f'{GOAL_RESPONSE_TIMEOUT_S:g} s')
            self.counts['failed'] += 1
            self.goal.cancel_requested = True   # cancel it if Nav2 accepts it late
            self.goal = None
            self.pending_event = ('failed', now)

        if self.state == 'waiting':
            gm, why = self.grid_map()
            pose, why_pose = self.robot_pose() if gm is not None else (None, '')
            if gm is None or pose is None:
                self.warn_waiting(now, why or why_pose)
                return
            if not self.nav.server_is_ready():
                self.warn_waiting(now, f'action server {NAV_ACTION} is not available')
                return
            self.state = 'exploring'
            self.t_started = now
            self.publish_status(ExploreStatus.EXPLORATION_STARTED)
            self.get_logger().info('Exploration started.')
            self.next_plan = 0.0

        if self.state != 'exploring':
            return

        event = None
        if self.pending_event is not None:
            kind, at = self.pending_event
            if self.map_rx <= at and now - at < self.map_wait:
                return                  # let the map catch up with where the robot stopped
            event = kind
            self.pending_event = None
        elif now < self.next_plan:
            return
        self.plan(now, event)

    def plan(self, now, event):
        self.next_plan = now + self.planner_period
        gm, why = self.grid_map()
        pose, why_pose = self.robot_pose() if gm is not None else (None, '')
        if gm is None or pose is None:
            self.warn_waiting(now, why or why_pose)
            return
        self.waiting_since = now
        fresh_map = self.map_rx > self.last_plan_map_rx
        self.last_plan_map_rx = self.map_rx
        active = (self.goal.x, self.goal.y) if self.goal is not None else None
        t0 = time.monotonic()
        d = self.planner.decide(gm, pose, event if event != 'canceled' else None, active)
        self.plan_times.append(time.monotonic() - t0)
        if self.visualize:
            self.publish_markers()

        if d.kind == 'done':
            if fresh_map or self.done_count == 0:
                self.done_count += 1
            if self.done_count >= self.done_confirmations:
                self.complete(d)
            else:
                self.get_logger().info(
                    f'No reachable frontier ({d.info}); confirming on a newer map '
                    f'({self.done_count}/{self.done_confirmations}).')
            return
        self.done_count = 0
        if d.kind == 'wait':
            self.warn_waiting(now, d.reason)
            return
        if d.kind == 'goal':
            self.send_goal(now, *d.goal, d.info)

    # ------------------------------------------------------------ Nav2 goals
    def send_goal(self, now, x, y, yaw, info):
        if not self.nav.server_is_ready():
            self.get_logger().warn(f'Action server {NAV_ACTION} is not available; retrying.')
            return
        if self.goal is not None:
            self.counts['replaced'] += 1
        self.goal_seq += 1
        g = SentGoal(self.goal_seq, x, y, yaw, now)
        self.goal = g
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = self.map_frame
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.counts['goals_sent'] += 1
        self.get_logger().info(
            f'Goal {g.seq}: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} deg {info}')
        future = self.nav.send_goal_async(msg)
        future.add_done_callback(lambda f, g=g: self.on_goal_response(g, f))

    def on_goal_response(self, g, future):
        try:
            handle = future.result()
        except Exception as exc:
            handle = None
            self.get_logger().warn(f'Goal {g.seq}: sending raised {exc!r}')
        if g is not self.goal:
            if handle is not None and handle.accepted and g.cancel_requested:
                handle.cancel_goal_async()
            return                      # replaced or dropped meanwhile
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'Goal {g.seq}: rejected by Nav2')
            self.counts['rejected'] += 1
            self.goal = None
            self.pending_event = ('failed', time.monotonic())
            return
        g.handle = handle
        if g.cancel_requested:
            handle.cancel_goal_async()
            return
        handle.get_result_async().add_done_callback(lambda f, g=g: self.on_goal_result(g, f))

    def on_goal_result(self, g, future):
        if g is not self.goal:
            return                      # a replaced goal: Nav2 aborts it on preemption
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().warn(f'Goal {g.seq}: reading the result raised {exc!r}')
            status = GoalStatus.STATUS_ABORTED
        now = time.monotonic()
        self.goal = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.counts['succeeded'] += 1
            kind = 'succeeded'
        elif status == GoalStatus.STATUS_CANCELED:
            if g.cancel_requested:
                return
            self.counts['canceled_by_others'] += 1
            kind = 'canceled'
        else:
            self.counts['failed'] += 1
            kind = 'failed'
        self.get_logger().info(f'Goal {g.seq}: {kind} after {now - g.sent:.1f} s')
        if self.state == 'exploring':
            self.pending_event = (kind, now)

    def cancel_goal(self, why):
        g = self.goal
        if g is None:
            return
        g.cancel_requested = True
        self.goal = None
        if g.handle is not None:
            g.handle.cancel_goal_async()
        self.get_logger().info(f'Goal {g.seq}: cancel requested ({why})')

    # ------------------------------------------------------------ completion
    def complete(self, d):
        self.cancel_goal('exploration complete')
        self.state = 'complete'
        self.get_logger().warn(COMPLETION_LINE)
        self.publish_status(ExploreStatus.EXPLORATION_COMPLETE)
        self.get_logger().info(self.stats_line(d.info))

    def stats_line(self, info=None):
        took = time.monotonic() - self.t_started if self.t_started is not None else 0.0
        pt = self.plan_times
        timing = (f'plan_s mean={np.mean(pt):.2f} max={np.max(pt):.2f} n={len(pt)}'
                  if pt else 'plan_s n=0')
        c = ' '.join(f'{k}={v}' for k, v in self.counts.items())
        s = ' '.join(f'{k}={v}' for k, v in self.planner.stats.items())
        return f'Exploration stats: duration_s={took:.0f} {c} {s} {timing} last={info or {}}'

    # ------------------------------------------------------------ markers
    def publish_markers(self):
        last = self.planner.last
        if last is None:
            return
        gm, opts, choice = last
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        def cells_marker(ns, xy, rgba):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = ns
            m.id = 0
            m.type = Marker.CUBE_LIST
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = gm.resolution
            m.scale.z = 0.02
            m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            m.points = [Point(x=float(px), y=float(py), z=0.0) for px, py in xy]
            return m

        others = [o.cells for k, o in enumerate(opts) if k != choice]
        if others:
            arr.markers.append(cells_marker('frontiers', gm.cells_to_world(np.concatenate(others)),
                                            (0.2, 0.4, 1.0, 0.8)))
        if choice is not None:
            o = opts[choice]
            arr.markers.append(cells_marker('target', gm.cells_to_world(o.cells),
                                            (0.1, 0.8, 0.1, 0.9)))
            vx, vy = gm.to_world(*o.viewpoint)
            arr.markers.append(cells_marker('viewpoint', [(vx, vy)], (0.1, 0.8, 0.1, 1.0)))
            arr.markers[-1].type = Marker.SPHERE_LIST
            arr.markers[-1].scale.x = arr.markers[-1].scale.y = arr.markers[-1].scale.z = 0.12
        if self.planner.excluded_xy:
            arr.markers.append(cells_marker('excluded', np.concatenate(self.planner.excluded_xy),
                                            (0.9, 0.1, 0.1, 0.8)))
        self.marker_pub.publish(arr)


def main(args=None):
    # Ctrl-C raises KeyboardInterrupt instead of shutting the context down, so
    # the node can still cancel its Nav2 goal before it exits.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = RoverExplorer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok() and node.goal is not None and node.goal.handle is not None:
                try:
                    future = node.goal.handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                except Exception:
                    pass
            if rclpy.ok():
                node.get_logger().info(node.stats_line())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
