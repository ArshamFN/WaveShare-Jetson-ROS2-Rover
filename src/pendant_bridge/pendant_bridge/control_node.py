"""Pendant control node.

Starts and stops the rover-bringup and rover-nav2 systemd user services,
publishes their state, saves SLAM maps with nav2_map_server, and runs a
two-press shutdown that stops the stack and then powers off the computer.
"""

import os
import re
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from slam_toolbox.srv import SaveMap
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

BRINGUP_UNIT = 'rover-bringup.service'
NAV2_UNIT = 'rover-nav2.service'

MAP_NAME_RE = re.compile(r'[A-Za-z0-9_-]{1,64}')

STATUS_TIMEOUT_S = 3.0
SYSTEMCTL_TIMEOUT_S = 10.0
PGREP_TIMEOUT_S = 3.0
SAVE_TIMEOUT_S = 30.0
STOP_TIMEOUT_S = 30.0
POWEROFF_TIMEOUT_S = 15.0

TOGGLE_DEBOUNCE_S = 2.0
SHUTDOWN_ARM_WINDOW_S = 5.0
SHUTDOWN_DELAY_S = 1.0

SAVE_OK = 0
SAVE_NO_MAP = 1
SAVE_FAILED = 255


def run(args, timeout):
    """Run a command without a shell. Returns (returncode, output, timed_out).

    The child gets its own process group so a timeout kills everything it
    spawned, not only the top-level process.
    """
    try:
        proc = subprocess.Popen(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            text=True,
            start_new_session=True,
        )
    except OSError as exc:
        return None, str(exc), False
    try:
        out, _ = proc.communicate(timeout=timeout)
        return proc.returncode, out or '', False
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        out, _ = proc.communicate()
        return proc.returncode, out or '', True


def summarise(output, limit=200):
    """Return the last non-empty output line, trimmed to one short line."""
    lines = [ln.strip() for ln in output.splitlines() if ln.strip()]
    if not lines:
        return ''
    last = lines[-1]
    return last if len(last) <= limit else last[:limit] + '...'


class PendantControl(Node):

    def __init__(self):
        super().__init__('pendant_control')
        self.declare_parameter('maps_dir', '~/ros2_ws/maps')
        self.maps_dir = os.path.expanduser(
            self.get_parameter('maps_dir').get_parameter_value().string_value)
        self.declare_parameter('allow_poweroff', False)

        self.toggle_lock = threading.Lock()
        self.last_toggle = {}
        self.shutdown_lock = threading.Lock()
        self.shutdown_armed_at = None
        self.shutdown_running = False

        group = ReentrantCallbackGroup()

        self.bringup_pub = self.create_publisher(Bool, '/pendant/bringup_active', 10)
        self.nav2_pub = self.create_publisher(Bool, '/pendant/nav2_active', 10)
        self.result_pub = self.create_publisher(String, '/pendant/control_result', 10)

        self.create_service(
            SetBool, '/pendant/set_bringup', self.on_set_bringup, callback_group=group)
        self.create_service(
            SetBool, '/pendant/set_nav2', self.on_set_nav2, callback_group=group)
        self.create_service(
            SaveMap, '/pendant/save_map', self.on_save_map, callback_group=group)
        self.create_service(
            Trigger, '/pendant/toggle_bringup', self.on_toggle_bringup,
            callback_group=group)
        self.create_service(
            Trigger, '/pendant/toggle_nav2', self.on_toggle_nav2, callback_group=group)
        self.create_service(
            Trigger, '/pendant/shutdown', self.on_shutdown, callback_group=group)

        self.create_timer(1.0, self.publish_status, callback_group=group)

        self.get_logger().info(f'pendant_control ready, maps_dir={self.maps_dir}')

    # ------------------------------------------------------------ helpers
    def report(self, text):
        self.get_logger().info(text)
        self.result_pub.publish(String(data=text))

    def unit_state(self, unit):
        """Return the raw systemctl is-active state, or 'unknown' if unreadable."""
        rc, out, timed_out = run(
            ['systemctl', '--user', 'is-active', unit], STATUS_TIMEOUT_S)
        state = out.strip()
        return 'unknown' if timed_out or not state else state

    def unit_active(self, unit):
        return self.unit_state(unit) == 'active'

    def process_running(self, pattern):
        rc, _, timed_out = run(['pgrep', '-f', pattern], PGREP_TIMEOUT_S)
        return not timed_out and rc == 0

    def systemctl(self, verb, unit, block=False, timeout=SYSTEMCTL_TIMEOUT_S):
        """Run a systemctl verb, non-blocking by default. Returns (ok, message)."""
        args = ['systemctl', '--user', verb, unit]
        if not block:
            args.insert(3, '--no-block')
        rc, out, timed_out = run(args, timeout)
        detail = summarise(out)
        if timed_out:
            return False, f'{verb} {unit} timed out after {timeout:.0f} s'
        if rc != 0:
            msg = f'{verb} {unit} failed (exit {rc})'
            return False, f'{msg}: {detail}' if detail else msg
        return True, f'{verb} {unit} {"done" if block else "requested"}'

    def start_bringup(self):
        """Checks shared by set_bringup true and toggle_bringup.

        Returns (ok, message, requested) where requested is True only when a
        start was actually issued.
        """
        if self.unit_active(BRINGUP_UNIT):
            return True, 'bringup already running', False
        if self.process_running('rover_driver_node'):
            return False, 'refused: bringup already running outside systemd', False
        ok, msg = self.systemctl('start', BRINGUP_UNIT)
        return ok, msg, ok

    def start_nav2(self):
        """Checks shared by set_nav2 true and toggle_nav2. Same return as start_bringup."""
        if not self.unit_active(BRINGUP_UNIT):
            return False, 'refused: bringup is not active', False
        if (not self.unit_active(NAV2_UNIT)
                and self.process_running('controller_server')):
            return False, 'refused: Nav2 already running outside systemd', False
        ok, msg = self.systemctl('start', NAV2_UNIT)
        return ok, msg, ok

    # ------------------------------------------------------------ status
    def publish_status(self):
        self.bringup_pub.publish(Bool(data=self.unit_active(BRINGUP_UNIT)))
        self.nav2_pub.publish(Bool(data=self.unit_active(NAV2_UNIT)))

    # ------------------------------------------------------------ services
    def on_set_bringup(self, request, response):
        if request.data:
            ok, msg, _ = self.start_bringup()
        else:
            ok, msg = self.systemctl('stop', BRINGUP_UNIT)
        response.success = ok
        response.message = msg
        self.report(f'set_bringup {request.data}: {msg}')
        return response

    def on_set_nav2(self, request, response):
        if request.data:
            ok, msg, _ = self.start_nav2()
        else:
            ok, msg = self.systemctl('stop', NAV2_UNIT)
        response.success = ok
        response.message = msg
        self.report(f'set_nav2 {request.data}: {msg}')
        return response

    def on_toggle_bringup(self, request, response):
        response.success, response.message = self.toggle(
            'toggle_bringup', BRINGUP_UNIT, 'bringup', self.start_bringup,
            'stopping bringup (Nav2 stops with it)', 'starting bringup')
        return response

    def on_toggle_nav2(self, request, response):
        response.success, response.message = self.toggle(
            'toggle_nav2', NAV2_UNIT, 'Nav2', self.start_nav2,
            'stopping Nav2', 'starting Nav2')
        return response

    def toggle(self, name, unit, label, start, stop_text, start_text):
        """Start or stop a unit based on its current state. Returns (success, message)."""
        with self.toggle_lock:
            now = time.monotonic()
            last = self.last_toggle.get(unit)
            if last is not None and now - last < TOGGLE_DEBOUNCE_S:
                ok, msg = False, 'refused: pressed too fast, wait 2 s'
            else:
                state = self.unit_state(unit)
                if state == 'active':
                    ok, msg = self.systemctl('stop', unit)
                    requested = ok
                    if ok:
                        msg = stop_text
                elif state in ('inactive', 'failed'):
                    ok, msg, requested = start()
                    if requested:
                        msg = start_text
                else:
                    ok, msg, requested = (
                        False, f'refused: {label} is {state}, try again shortly', False)
                ok = requested
                if requested:
                    self.last_toggle[unit] = now
        self.report(f'{name}: {msg}')
        return ok, msg

    def on_shutdown(self, request, response):
        with self.shutdown_lock:
            now = time.monotonic()
            if self.shutdown_running:
                ok, msg = False, 'shutdown already in progress'
            elif (self.shutdown_armed_at is None
                    or now - self.shutdown_armed_at > SHUTDOWN_ARM_WINDOW_S):
                self.shutdown_armed_at = now
                ok, msg = False, 'ARMED: press SHUTDOWN again within 5 s to power off'
            else:
                self.shutdown_armed_at = None
                self.shutdown_running = True
                ok, msg = True, 'shutting down: stopping Nav2 and bringup, then powering off'
                threading.Thread(target=self.shutdown_sequence, daemon=True).start()
        response.success = ok
        response.message = msg
        self.report(f'shutdown: {msg}')
        return response

    def shutdown_sequence(self):
        """Stop Nav2 and bringup, then power off if allowed. Runs in its own thread."""
        finished = True
        try:
            time.sleep(SHUTDOWN_DELAY_S)
            for unit in (NAV2_UNIT, BRINGUP_UNIT):
                ok, msg = self.systemctl('stop', unit, block=True, timeout=STOP_TIMEOUT_S)
                self.report(f'shutdown: {msg}')
            self.report(
                f'shutdown: {NAV2_UNIT} is {self.unit_state(NAV2_UNIT)}, '
                f'{BRINGUP_UNIT} is {self.unit_state(BRINGUP_UNIT)}')
            allow = self.get_parameter(
                'allow_poweroff').get_parameter_value().bool_value
            if not allow:
                self.report('poweroff skipped: allow_poweroff is false, units stopped')
                return
            rc, out, timed_out = run(
                ['sudo', '-n', '/usr/bin/systemctl', 'poweroff'], POWEROFF_TIMEOUT_S)
            if timed_out:
                self.report(
                    f'poweroff timed out after {POWEROFF_TIMEOUT_S:.0f} s: {out}')
            elif rc != 0:
                self.report(f'poweroff failed (exit {rc}): {out}')
            else:
                # The machine is going down; keep refusing further presses.
                finished = False
                self.report('poweroff requested')
        except Exception as exc:
            self.report(f'shutdown sequence error: {exc}')
        finally:
            if finished:
                with self.shutdown_lock:
                    self.shutdown_running = False

    def on_save_map(self, request, response):
        name = request.name.data
        response.result, msg = self.save_map(name)
        self.report(f'save_map {name!r}: {msg}')
        return response

    def save_map(self, name):
        if not MAP_NAME_RE.fullmatch(name):
            return SAVE_FAILED, 'refused: name must match [A-Za-z0-9_-]{1,64}'
        if not self.unit_active(BRINGUP_UNIT):
            return SAVE_FAILED, 'refused: bringup is not active'

        base = os.path.join(self.maps_dir, name)
        yaml_path, pgm_path = base + '.yaml', base + '.pgm'
        existing = [p for p in (yaml_path, pgm_path) if os.path.exists(p)]
        if existing:
            return SAVE_FAILED, f'refused: {", ".join(existing)} already exists'

        rc, out, timed_out = run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', base,
             '--ros-args', '-p', 'map_subscribe_transient_local:=true'],
            SAVE_TIMEOUT_S)
        detail = summarise(out)
        have_yaml, have_pgm = os.path.exists(yaml_path), os.path.exists(pgm_path)

        if have_yaml and have_pgm:
            return SAVE_OK, f'saved {yaml_path} and {pgm_path}'
        if timed_out:
            return SAVE_FAILED, f'map saver timed out after {SAVE_TIMEOUT_S:.0f} s'
        if rc is None:
            return SAVE_FAILED, f'map saver could not start: {detail}'
        if not have_yaml and not have_pgm:
            msg = f'map saver exited {rc} and wrote no files'
            return SAVE_NO_MAP, f'{msg}: {detail}' if detail else msg
        return SAVE_FAILED, f'map saver exited {rc} and wrote only one file: {detail}'


def main(args=None):
    rclpy.init(args=args)
    node = PendantControl()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
