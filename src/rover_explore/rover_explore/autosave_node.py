"""Exploration autosave node.

Saves the SLAM map while the explorer explores (the rover_explorer node, or
explore_lite; both publish the same /explore/status messages): a checkpoint
every few minutes while exploration is running, and a final map after the
explorer reports exploration_complete and the rover has finished driving back
to its start pose (or the return was skipped, failed or canceled), so the
save happens with the rover at rest rather than during the drive home. Maps
are saved through the pendant's /pendant/save_map service. Checkpoint maps are
moved out of maps/ into the run folder, so maps/ only receives the final map.
Every save also serializes the slam_toolbox pose graph into the run folder,
and the node logs battery voltage and every event to CSV files in that folder.

The node also brings the rover back to where it started. At startup, before
the explorer sends its first goal, it records the robot's pose in the map
frame from TF. A few seconds after exploration_complete it sends one Nav2
NavigateToPose goal to that pose, and retries a goal that Nav2 rejects or
aborts a limited number of times; the final map is saved once that has
ended. explore_lite's own return_to_init stays off: explore_lite sends its
return goal right after asking Nav2 to cancel all goals, and that cancel can
catch the return goal too.

The node never publishes velocity commands and never pauses or resumes
exploration. Stopping exploration (explore/resume false) makes the explorer
publish exploration_paused, even after completion; the node then cancels a
return in progress (explore_lite also cancels every Nav2 goal itself), does
not retry it, and drops a return that has not been sent yet. If the node
itself is stopped before the return ends, there is no final save; the latest
checkpoint and the live SLAM map remain.

For comparing runs, the node also counts Nav2's own log lines on /rosout
(goals begun, succeeded, failed, canceled and preempted, progress failures and
recovery behaviours) and integrates the distance driven from /odom. It
records them when exploration completes and when the node stops. /rosout is
read best effort, so the counts can miss lines under heavy load.
"""

import csv
import datetime
import math
import os
import shutil
import time

from action_msgs.msg import GoalStatus
from explore_lite_msgs.msg import ExploreStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import Float32, String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

SAVE_SERVICE = '/pendant/save_map'
SERIALIZE_SERVICE = '/slam_toolbox/serialize_map'
STATUS_TOPIC = '/explore/status'
BATTERY_TOPIC = '/battery_voltage'
ROSOUT_TOPIC = '/rosout'
PENDANT_RESULT_TOPIC = '/pendant/control_result'
ODOM_TOPIC = '/odom'
NAV_ACTION = '/navigate_to_pose'

# A start pose read from a transform older than this is not trusted.
START_POSE_MAX_AGE_S = 2.0
# Time Nav2 gets to accept or reject a return goal.
GOAL_RESPONSE_TIMEOUT_S = 10.0

# Result codes returned by /pendant/save_map (pendant_bridge control_node).
SAVE_RESULTS = {0: 'saved', 1: 'no map received', 255: 'failed'}
SAVE_OK = 0
SERIALIZE_OK = 0

EXPLORING = (ExploreStatus.EXPLORATION_STARTED, ExploreStatus.EXPLORATION_IN_PROGRESS)
# Return stages during which the final save waits.
RETURN_ACTIVE = ('scheduled', 'sending', 'driving')

# The explorers log why they published exploration_complete; explore_lite has
# two different reasons and only these lines tell them apart.
COMPLETION_CAUSES = {
    'No frontiers found, stopping.': 'no_frontiers',
    'All frontiers traversed/tried out, stopping.': 'all_blacklisted',
    'No reachable frontiers left, stopping.': 'no_reachable_frontiers',
}
EXPLORER_STATS_PREFIX = 'Exploration stats:'

# Nav2 (Humble) log lines worth counting, matched at the start of the message.
NAV2_LINES = [
    ('begun', 'Begin navigating'),
    ('succeeded', 'Goal succeeded'),
    ('failed', 'Goal failed'),
    ('canceled', 'Goal canceled'),
    ('preempted', 'Received goal preemption'),
    ('no_progress', 'Failed to make progress'),
    ('spin', 'Running spin'),
    ('backup', 'Running backup'),
    ('wait', 'Running wait'),
]
# A jump larger than this between two /odom messages is a reset, not driving.
ODOM_MAX_STEP_M = 0.5

TICK_PERIOD_S = 0.5
BATTERY_PERIOD_S = 1.0
NO_STATUS_WARNING_S = 60.0

EVENT_FIELDS = ['iso_time', 'elapsed_s', 'event', 'detail']
BATTERY_FIELDS = ['iso_time', 'elapsed_s', 'last_v', 'min_v', 'samples']


def iso_now():
    return datetime.datetime.now().astimezone().isoformat(timespec='seconds')


class CsvLog:
    """Append-only CSV file, flushed to disk after every row."""

    def __init__(self, path, header):
        self._file = open(path, 'x', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file, lineterminator='\n')
        self.write(header)

    def write(self, row):
        self._writer.writerow(row)
        self._file.flush()
        os.fsync(self._file.fileno())

    def close(self):
        if not self._file.closed:
            self._file.close()


class Operation:
    """One save in progress: a map save, then a pose graph serialization."""

    def __init__(self, kind, name, base):
        self.kind = kind          # 'checkpoint' or 'final'
        self.name = name          # map name sent to /pendant/save_map
        self.base = base          # file name stem for the run folder
        self.stage = None         # 'save', 'wait_retry' or 'serialize'
        self.attempt = 0          # final save attempt number, 0 is the first
        self.future = None
        self.started = 0.0
        self.deadline = 0.0
        self.retry_at = 0.0
        self.serialize_path = ''

    def describe(self):
        return f'{self.kind} {self.name} at stage {self.stage}'


class ExploreAutosave(Node):

    def __init__(self):
        super().__init__('explore_autosave')
        self.t0 = time.monotonic()

        self.runs_dir = os.path.expanduser(
            self.declare_parameter('runs_dir', '~/rover_runs').value)
        self.maps_dir = os.path.expanduser(
            self.declare_parameter('maps_dir', '~/ros2_ws/maps').value)
        self.checkpoint_period = float(
            self.declare_parameter('checkpoint_period_s', 300.0).value)
        self.final_retries = int(self.declare_parameter('final_retries', 3).value)
        self.retry_delay = float(self.declare_parameter('final_retry_delay_s', 10.0).value)
        self.save_timeout = float(self.declare_parameter('save_timeout_s', 45.0).value)
        self.serialize_enabled = bool(
            self.declare_parameter('serialize_pose_graph', True).value)
        self.serialize_timeout = float(
            self.declare_parameter('serialize_timeout_s', 30.0).value)
        self.return_enabled = bool(self.declare_parameter('return_to_start', True).value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.base_frame = str(self.declare_parameter('robot_base_frame', 'base_link').value)
        self.start_pose_timeout = float(
            self.declare_parameter('start_pose_timeout_s', 15.0).value)
        self.return_delay = float(self.declare_parameter('return_delay_s', 2.0).value)
        self.return_retries = int(self.declare_parameter('return_retries', 2).value)
        self.return_retry_delay = float(
            self.declare_parameter('return_retry_delay_s', 10.0).value)
        self.return_timeout = float(self.declare_parameter('return_timeout_s', 600.0).value)

        if self.checkpoint_period <= 0.0:
            raise ValueError('checkpoint_period_s must be greater than 0')
        if self.final_retries < 0:
            raise ValueError('final_retries must be 0 or more')
        if self.retry_delay < 0.0 or self.save_timeout <= 0.0 or self.serialize_timeout <= 0.0:
            raise ValueError('final_retry_delay_s must be 0 or more and both timeouts above 0')
        if not os.path.isdir(self.maps_dir):
            raise ValueError(f'maps_dir {self.maps_dir} is not a directory')
        if not self.map_frame or not self.base_frame:
            raise ValueError('map_frame and robot_base_frame must not be empty')
        if self.return_retries < 0:
            raise ValueError('return_retries must be 0 or more')
        if (self.start_pose_timeout <= 0.0 or self.return_timeout <= 0.0
                or self.return_delay < 0.0 or self.return_retry_delay < 0.0):
            raise ValueError(
                'start_pose_timeout_s and return_timeout_s must be above 0, '
                'return_delay_s and return_retry_delay_s 0 or more')

        self.run_id, self.run_dir = self.make_run_dir()
        self.events = CsvLog(os.path.join(self.run_dir, 'events.csv'), EVENT_FIELDS)
        self.battery = CsvLog(os.path.join(self.run_dir, 'battery.csv'), BATTERY_FIELDS)

        self.status = None
        self.checkpoint_clock = None   # monotonic time the checkpoint countdown restarted
        self.checkpoint_count = 0
        self.final_requested = False
        self.final_wait_logged = False
        self.final_done = False
        self.op = None
        self.no_status_warned = False

        self.batt_last = None
        self.batt_min = None
        self.batt_samples = 0

        # Run metrics.
        self.nav2_counts = {key: 0 for key, _ in NAV2_LINES}
        self.odom_last = None
        self.odom_distance = 0.0
        self.explore_t0 = None          # monotonic time exploration started
        self.explore_d0 = 0.0           # odom distance when it started
        self.explore_counts0 = None
        self.summary_logged = False

        # Return to the start pose.
        self.start_pose = None         # geometry_msgs/Transform of the robot in the map frame
        self.start_pose_error = 'no transform received yet'
        self.start_pose_given_up = False
        self.start_pose_deadline = time.monotonic() + self.start_pose_timeout
        self.ret_stage = None          # None, 'scheduled', 'sending', 'driving' or 'done'
        self.ret_attempt = 0
        self.ret_at = 0.0
        self.ret_started = 0.0
        self.ret_deadline = 0.0
        self.ret_future = None
        self.ret_handle = None
        self.ret_stop_requested = False
        self.tf_buffer = None
        self.tf_listener = None
        self.nav_client = None
        if self.return_enabled:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.nav_client = ActionClient(self, NavigateToPose, NAV_ACTION)

        self.save_client = self.create_client(SaveMap, SAVE_SERVICE)
        self.serialize_client = self.create_client(SerializePoseGraph, SERIALIZE_SERVICE)

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Best effort, so this reader can never hold up any other node's logging.
        rosout_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(ExploreStatus, STATUS_TOPIC, self.on_status, status_qos)
        self.create_subscription(Float32, BATTERY_TOPIC, self.on_battery, 10)
        self.create_subscription(Log, ROSOUT_TOPIC, self.on_rosout, rosout_qos)
        self.create_subscription(String, PENDANT_RESULT_TOPIC, self.on_pendant_result, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 20)
        self.create_timer(BATTERY_PERIOD_S, self.on_battery_timer)
        self.create_timer(TICK_PERIOD_S, self.tick)

        self.event('node_started', (
            f'run_id={self.run_id} run_dir={self.run_dir} maps_dir={self.maps_dir} '
            f'checkpoint_period_s={self.checkpoint_period:g} '
            f'final_retries={self.final_retries} '
            f'final_retry_delay_s={self.retry_delay:g} '
            f'serialize_pose_graph={self.serialize_enabled} '
            f'return_to_start={self.return_enabled} '
            f'frames={self.map_frame}->{self.base_frame} '
            f'return_delay_s={self.return_delay:g} '
            f'return_retries={self.return_retries} '
            f'return_retry_delay_s={self.return_retry_delay:g} '
            f'return_timeout_s={self.return_timeout:g}'))

    # ------------------------------------------------------------ helpers
    def elapsed(self):
        return time.monotonic() - self.t0

    def make_run_dir(self):
        os.makedirs(self.runs_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        for suffix in ['', '-2', '-3', '-4', '-5']:
            run_id = f'explore_{stamp}{suffix}'
            path = os.path.join(self.runs_dir, run_id)
            try:
                os.mkdir(path)
            except FileExistsError:
                continue
            return run_id, path
        raise RuntimeError(f'could not create a new run folder in {self.runs_dir}')

    def event(self, name, detail='', warn=False):
        self.events.write([iso_now(), f'{self.elapsed():.1f}', name, detail])
        text = f'{name}: {detail}' if detail else name
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    # ------------------------------------------------------------ inputs
    def on_status(self, msg):
        status = msg.status
        self.event('status', status)
        self.status = status
        if status in EXPLORING and self.checkpoint_clock is None:
            self.checkpoint_clock = time.monotonic()
        if status in EXPLORING and self.explore_t0 is None:
            self.explore_t0 = time.monotonic()
            self.explore_d0 = self.odom_distance
            self.explore_counts0 = dict(self.nav2_counts)
        if status == ExploreStatus.EXPLORATION_COMPLETE and not self.summary_logged:
            self.summary_logged = True
            self.event('exploration_summary', self.metrics(since_start=True))
        if status == ExploreStatus.EXPLORATION_COMPLETE and self.ret_stage is None:
            self.schedule_return()
        if status == ExploreStatus.EXPLORATION_PAUSED:
            self.stop_return()
        if status == ExploreStatus.EXPLORATION_COMPLETE and not self.final_requested:
            self.final_requested = True
            self.event('final_requested', 'first exploration_complete received')
            self.tick()

    def on_rosout(self, msg):
        if msg.name == self.get_name():
            return
        cause = COMPLETION_CAUSES.get(msg.msg)
        if cause is not None:
            self.event('completion_cause', f'{cause} (logged by {msg.name})')
            return
        if msg.msg.startswith(EXPLORER_STATS_PREFIX):
            self.event('explorer_stats', f'{msg.msg[len(EXPLORER_STATS_PREFIX):].strip()} '
                                         f'(logged by {msg.name})')
            return
        for key, prefix in NAV2_LINES:
            if msg.msg.startswith(prefix):
                self.nav2_counts[key] += 1
                return

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.odom_last is not None:
            step = math.hypot(p.x - self.odom_last[0], p.y - self.odom_last[1])
            if step <= ODOM_MAX_STEP_M:
                self.odom_distance += step
        self.odom_last = (p.x, p.y)

    def metrics(self, since_start):
        """Distance and Nav2 counts, for the whole node run or since exploration started."""
        if since_start and self.explore_t0 is not None:
            took = f'{time.monotonic() - self.explore_t0:.0f}'
            dist = self.odom_distance - self.explore_d0
            counts = {k: v - self.explore_counts0[k] for k, v in self.nav2_counts.items()}
        else:
            took = f'{self.elapsed():.0f}'
            dist = self.odom_distance
            counts = self.nav2_counts
        text = ' '.join(f'nav2_{k}={v}' for k, v in counts.items())
        return f'duration_s={took} odom_distance_m={dist:.2f} {text}'

    def on_pendant_result(self, msg):
        # The pendant explains a refused or failed save only on this topic.
        if msg.data.startswith('save_map '):
            self.event('pendant', msg.data)

    def on_battery(self, msg):
        volts = float(msg.data)
        if not math.isfinite(volts):
            return
        self.batt_last = volts
        self.batt_min = volts if self.batt_min is None else min(self.batt_min, volts)
        self.batt_samples += 1

    def on_battery_timer(self):
        last = '' if self.batt_last is None else f'{self.batt_last:.2f}'
        low = '' if self.batt_min is None else f'{self.batt_min:.2f}'
        self.battery.write([iso_now(), f'{self.elapsed():.1f}', last, low, self.batt_samples])
        self.batt_last = None
        self.batt_min = None
        self.batt_samples = 0

    # ------------------------------------------------------------ scheduling
    def tick(self):
        now = time.monotonic()
        if (self.status is None and not self.no_status_warned
                and now - self.t0 >= NO_STATUS_WARNING_S):
            self.no_status_warned = True
            self.event('no_status_yet', (
                f'no message on {STATUS_TOPIC} after {NO_STATUS_WARNING_S:.0f} s; '
                "check the explorer's log"), warn=True)

        if self.return_enabled:
            self.update_start_pose(now)
            self.advance_return(now)

        if self.op is not None:
            self.advance(now)

        checkpoint_due = (
            not self.final_requested
            and self.status in EXPLORING
            and self.checkpoint_clock is not None
            and now - self.checkpoint_clock >= self.checkpoint_period)

        if self.op is not None:
            if checkpoint_due:
                self.checkpoint_clock = now
                self.event('checkpoint_skipped', f'busy with {self.op.describe()}', warn=True)
            return

        if self.final_requested and not self.final_done:
            if self.ret_stage in RETURN_ACTIVE:
                if not self.final_wait_logged:
                    self.final_wait_logged = True
                    self.event('final_waiting',
                               'saving the final map once the return to the start pose has ended')
                return
            self.start_final(now)
        elif checkpoint_due:
            self.start_checkpoint(now)

    def start_checkpoint(self, now):
        self.checkpoint_clock = now
        self.checkpoint_count += 1
        name = f'{self.run_id}_ckpt{self.checkpoint_count:02d}'
        self.op = Operation('checkpoint', name, name)
        self.event('checkpoint_start', name)
        self.begin_save(now)

    def start_final(self, now):
        name = f'{self.run_id}_final'
        self.op = Operation('final', name, name)
        self.event('final_start', name)
        self.begin_save(now)

    # ------------------------------------------------------------ operation stages
    def begin_save(self, now):
        op = self.op
        op.stage = 'save'
        op.started = now
        if not self.save_client.service_is_ready():
            self.save_finished(now, False, f'service {SAVE_SERVICE} is not available')
            return
        request = SaveMap.Request()
        request.name.data = op.name
        op.future = self.save_client.call_async(request)
        op.deadline = now + self.save_timeout
        self.event('save_request', op.name)

    def begin_serialize(self, now):
        op = self.op
        if not self.serialize_enabled:
            self.finish(now)
            return
        op.stage = 'serialize'
        op.started = now
        op.serialize_path = os.path.join(self.run_dir, op.base)
        if not self.serialize_client.service_is_ready():
            self.event('serialize_failed', (
                f'{op.serialize_path}: service {SERIALIZE_SERVICE} is not available'), warn=True)
            self.finish(now)
            return
        request = SerializePoseGraph.Request()
        request.filename = op.serialize_path
        op.future = self.serialize_client.call_async(request)
        op.deadline = now + self.serialize_timeout
        self.event('serialize_request', op.serialize_path)

    def advance(self, now):
        op = self.op
        if op.stage == 'save':
            if op.future.done():
                ok, detail = self.read_save_result(op.future)
                self.save_finished(now, ok, detail)
            elif now >= op.deadline:
                self.drop_request(self.save_client, op.future)
                self.save_finished(now, False, f'no response within {self.save_timeout:g} s')
        elif op.stage == 'wait_retry':
            if now >= op.retry_at:
                op.attempt += 1
                op.name = f'{self.run_id}_final_r{op.attempt}'
                self.event('final_retry', f'attempt {op.attempt + 1} as {op.name}')
                self.begin_save(now)
        elif op.stage == 'serialize':
            if op.future.done():
                self.serialize_finished(now, op.future)
            elif now >= op.deadline:
                self.drop_request(self.serialize_client, op.future)
                self.event('serialize_failed', (
                    f'{op.serialize_path}: no response within {self.serialize_timeout:g} s'),
                    warn=True)
                self.finish(now)

    def read_save_result(self, future):
        try:
            response = future.result()
        except Exception as exc:
            return False, f'call raised {exc!r}'
        if response is None:
            return False, 'call returned no response'
        code = int(response.result)
        return code == SAVE_OK, f'result {code} ({SAVE_RESULTS.get(code, "unknown")})'

    def drop_request(self, client, future):
        try:
            client.remove_pending_request(future)
        except Exception:
            pass

    def save_finished(self, now, ok, detail):
        op = self.op
        took = now - op.started
        if ok:
            self.event('save_ok', f'{op.name}: {detail}, {took:.1f} s')
        else:
            self.event('save_failed', f'{op.name}: {detail}, {took:.1f} s', warn=True)

        if op.kind == 'checkpoint':
            # After a failed save, move whatever the map saver left behind.
            self.move_checkpoint(op.name, only_existing=not ok)
            self.begin_serialize(now)
            return

        if ok:
            self.event('final_saved', os.path.join(self.maps_dir, op.name + '.yaml'))
            self.begin_serialize(now)
        elif op.attempt < self.final_retries:
            op.stage = 'wait_retry'
            op.retry_at = now + self.retry_delay
            self.event('final_retry_scheduled', (
                f'attempt {op.attempt + 2} of {self.final_retries + 1} '
                f'in {self.retry_delay:g} s'), warn=True)
        else:
            self.event('final_failed', (
                f'all {self.final_retries + 1} attempts failed; '
                'save the map from the pendant'), warn=True)
            self.begin_serialize(now)

    def move_checkpoint(self, name, only_existing=False):
        moved = []
        for ext in ('.pgm', '.yaml'):
            src = os.path.join(self.maps_dir, name + ext)
            dst = os.path.join(self.run_dir, name + ext)
            if only_existing and not os.path.exists(src):
                continue
            try:
                shutil.move(src, dst)
                moved.append(dst)
            except OSError as exc:
                self.event('checkpoint_move_failed', f'{src}: {exc}', warn=True)
        if moved:
            self.event('checkpoint_moved', ' and '.join(moved))

    def serialize_finished(self, now, future):
        op = self.op
        took = now - op.started
        try:
            response = future.result()
        except Exception as exc:
            response = None
            detail = f'call raised {exc!r}'
        else:
            detail = 'call returned no response'
        files = [op.serialize_path + '.posegraph', op.serialize_path + '.data']
        missing = [f for f in files if not os.path.isfile(f) or os.path.getsize(f) == 0]
        if response is not None:
            code = int(response.result)
            if code == SERIALIZE_OK and not missing:
                sizes = ', '.join(f'{os.path.getsize(f)} B' for f in files)
                self.event('serialize_ok', f'{op.serialize_path} ({sizes}), {took:.1f} s')
                self.finish(now)
                return
            detail = f'result {code}'
            if missing:
                detail += f', missing or empty: {" ".join(missing)}'
        self.event('serialize_failed', f'{op.serialize_path}: {detail}, {took:.1f} s', warn=True)
        self.finish(now)

    def finish(self, now):
        op = self.op
        self.op = None
        if op.kind == 'final':
            self.final_done = True
            self.event('final_done', 'no further saves this run')
        else:
            self.event('checkpoint_done', op.name)

    # ------------------------------------------------------------ return to start
    def update_start_pose(self, now):
        """Record the robot's pose in the map frame once, as early as possible."""
        if self.start_pose is not None or self.start_pose_given_up:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException as exc:
            tf = None
            self.start_pose_error = ' '.join(str(exc).split())[:200] or type(exc).__name__
        if tf is not None:
            age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            if age <= START_POSE_MAX_AGE_S:
                self.start_pose = tf.transform
                self.stop_tf()
                self.event('start_pose', (
                    f'{self.describe_pose(tf.transform)} in {self.map_frame} '
                    f'(transform age {age:.2f} s)'))
                return
            self.start_pose_error = (
                f'newest {self.map_frame}->{self.base_frame} transform is {age:.1f} s old')
        if now >= self.start_pose_deadline:
            self.start_pose_given_up = True
            self.stop_tf()
            self.event('start_pose_failed', (
                f'no usable {self.map_frame}->{self.base_frame} transform within '
                f'{self.start_pose_timeout:g} s ({self.start_pose_error}); '
                'the rover will not drive back after exploration'), warn=True)

    def stop_tf(self):
        # The start pose is only read once, so stop receiving /tf afterwards.
        if self.tf_listener is not None:
            self.tf_listener.unregister()

    @staticmethod
    def describe_pose(transform):
        t, q = transform.translation, transform.rotation
        yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        return f'x={t.x:.2f} y={t.y:.2f} yaw={yaw:.0f} deg'

    def schedule_return(self):
        if not self.return_enabled:
            self.ret_stage = 'done'
            self.event('return_skipped', 'return_to_start is false')
            return
        self.ret_stage = 'scheduled'
        self.ret_at = time.monotonic() + self.return_delay
        self.event('return_scheduled', f'in {self.return_delay:g} s')

    def stop_return(self):
        """Exploration was stopped: drop a return not sent yet, cancel one under way."""
        if self.ret_stage == 'scheduled':
            self.ret_stage = 'done'
            self.event('return_dropped', 'exploration was stopped before the goal was sent')
        elif self.ret_stage in ('sending', 'driving') and not self.ret_stop_requested:
            self.ret_stop_requested = True
            self.event('return_stop_requested', 'exploration was stopped; canceling the return')
            if self.ret_stage == 'driving':
                self.cancel_return_goal()

    def advance_return(self, now):
        stage = self.ret_stage
        if stage == 'scheduled':
            if now >= self.ret_at:
                self.send_return(now)
        elif stage == 'sending':
            if self.ret_future.done():
                self.goal_response(now)
            elif now >= self.ret_deadline:
                self.return_attempt_failed(now, (
                    f'Nav2 did not accept or reject the goal within '
                    f'{GOAL_RESPONSE_TIMEOUT_S:g} s'))
        elif stage == 'driving':
            if self.ret_future.done():
                self.goal_result(now)
            elif now >= self.ret_deadline:
                self.cancel_return_goal()
                self.ret_stage = 'done'
                self.event('return_failed', (
                    f'goal still running after {self.return_timeout:g} s, canceled it; '
                    'drive the rover back manually'), warn=True)

    def send_return(self, now):
        if self.start_pose is None:
            if not self.start_pose_given_up:
                return  # still reading the start pose; try again next tick
            self.ret_stage = 'done'
            self.event('return_skipped', 'no start pose was recorded', warn=True)
            return
        self.ret_attempt += 1
        if not self.nav_client.server_is_ready():
            self.return_attempt_failed(now, f'action server {NAV_ACTION} is not available')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.start_pose.translation.x
        goal.pose.pose.position.y = self.start_pose.translation.y
        goal.pose.pose.orientation = self.start_pose.rotation
        self.ret_future = self.nav_client.send_goal_async(goal)
        self.ret_stage = 'sending'
        self.ret_started = now
        self.ret_deadline = now + GOAL_RESPONSE_TIMEOUT_S
        self.event('return_sent', (
            f'attempt {self.ret_attempt} of {self.return_retries + 1}: '
            f'{self.describe_pose(self.start_pose)}'))

    def goal_response(self, now):
        try:
            handle = self.ret_future.result()
        except Exception as exc:
            self.return_attempt_failed(now, f'sending the goal raised {exc!r}')
            return
        if handle is None or not handle.accepted:
            self.return_attempt_failed(now, 'Nav2 rejected the goal')
            return
        self.ret_handle = handle
        self.ret_future = handle.get_result_async()
        self.ret_stage = 'driving'
        self.ret_deadline = now + self.return_timeout
        self.event('return_accepted', f'attempt {self.ret_attempt}')
        if self.ret_stop_requested:
            self.cancel_return_goal()

    def goal_result(self, now):
        took = now - self.ret_started
        try:
            status = self.ret_future.result().status
        except Exception as exc:
            self.return_attempt_failed(now, f'reading the result raised {exc!r}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.ret_stage = 'done'
            self.event('return_ok', (
                f'reached the start pose on attempt {self.ret_attempt}, {took:.1f} s'))
        elif status == GoalStatus.STATUS_CANCELED:
            self.ret_stage = 'done'
            self.event('return_canceled', (
                f'the goal was canceled after {took:.1f} s; not retrying'), warn=True)
        elif status == GoalStatus.STATUS_ABORTED:
            self.return_attempt_failed(now, f'Nav2 aborted the goal after {took:.1f} s')
        else:
            self.return_attempt_failed(now, f'goal ended with status {status} after {took:.1f} s')

    def cancel_return_goal(self):
        if self.ret_handle is None:
            return
        try:
            self.ret_handle.cancel_goal_async()
        except Exception as exc:
            self.event('return_cancel_failed', repr(exc), warn=True)

    def return_attempt_failed(self, now, detail):
        attempts = self.return_retries + 1
        self.ret_future = None
        self.ret_handle = None
        self.event('return_attempt_failed', f'attempt {self.ret_attempt} of {attempts}: {detail}',
                   warn=True)
        if self.ret_stop_requested:
            self.ret_stage = 'done'
            self.event('return_dropped', 'exploration was stopped; not retrying')
        elif self.ret_attempt < attempts:
            self.ret_stage = 'scheduled'
            self.ret_at = now + self.return_retry_delay
            self.event('return_retry_scheduled', (
                f'attempt {self.ret_attempt + 1} of {attempts} '
                f'in {self.return_retry_delay:g} s'), warn=True)
        else:
            self.ret_stage = 'done'
            self.event('return_failed', (
                f'all {attempts} attempts failed; drive the rover back manually'), warn=True)

    # ------------------------------------------------------------ shutdown
    def close_logs(self):
        # The ROS context is already shut down here, so this goes to the CSV only.
        detail = 'idle' if self.op is None else f'interrupted {self.op.describe()}'
        detail += f'; whole run: {self.metrics(since_start=False)}'
        if self.ret_stage == 'scheduled':
            detail += '; return goal not sent yet'
        elif self.ret_stage == 'sending':
            detail += '; return goal sent, Nav2 had not answered'
        elif self.ret_stage == 'driving':
            detail += '; return goal still running in Nav2'
        try:
            self.events.write([iso_now(), f'{self.elapsed():.1f}', 'node_stopping', detail])
        finally:
            self.events.close()
            self.battery.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExploreAutosave()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close_logs()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
