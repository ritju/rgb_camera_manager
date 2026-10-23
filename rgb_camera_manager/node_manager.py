import ctypes
import os
import subprocess
import time
from signal import SIGINT, SIGKILL

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from capella_ros_service_interfaces.srv import SwitchResolution
from capella_ros_service_interfaces.msg import RgbCameraResolution

LAUNCH_LOW = 'rgb_camera_back_640x480.launch.py'
LAUNCH_HIGH = 'rgb_camera_back_1280x1024.launch.py'
SHUTDOWN_TIMEOUT = 10.0
KILL_TIMEOUT = 2.0
REAP_INTERVAL = 1.0
PR_SET_CHILD_SUBREAPER = 36
# Empirical: resolution switches closer than ~15s can OOM the host (UVC
# REQBUFS under multi-camera load). Enforce a 10s floor; frequent switches
# are refused with success=False instead of actually toggling.
MIN_SWITCH_INTERVAL_SEC = 10.0
# /proc/<pid>/stat states that cannot hold a V4L2 fd or accept signals.
_DEAD_STATES = frozenset(('Z', 'X', 'x'))


def _switch_too_soon(last_switch_mono, now_mono, min_interval=MIN_SWITCH_INTERVAL_SEC):
    """True if a mode change is requested too soon after the previous one.

    last_switch_mono is 0.0/None when no switch has completed yet — the
    first real switch is always allowed.
    """
    if not last_switch_mono:
        return False
    return (now_mono - last_switch_mono) < min_interval


def _enable_child_subreaper():
    """Ask the kernel to reparent orphans to us (Linux PR_SET_CHILD_SUBREAPER)."""
    try:
        libc = ctypes.CDLL('libc.so.6', use_errno=True)
        return libc.prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) == 0
    except Exception:
        return False


def _read_stat_fields(pid):
    """Return (state, pgrp, sid) from /proc/<pid>/stat, or None if unreadable."""
    try:
        with open('/proc/%d/stat' % pid, 'rb') as handle:
            raw = handle.read().decode('utf-8', 'replace')
        # comm (field 2) may contain spaces/parens: split after last ') '
        rest = raw.rsplit(') ', 1)[1].split()
        return rest[0], int(rest[2]), int(rest[3])
    except (OSError, IndexError, ValueError):
        return None


class NodeManagerService(Node):

    def __init__(self):
        super().__init__('rgb_camera_manager_server')

        # Track all processes we spawn (e.g., multiple ros2 launch commands)
        self.current_procs = []
        # pid -> Popen, so a global waitpid sweep can sync returncode
        self._tracked = {}
        # Monotonic time of the last completed mode-changing switch (0 = none).
        self._last_switch_mono = 0.0

        self._subreaper = _enable_child_subreaper()
        if self._subreaper:
            self.get_logger().info('child subreaper: on')
        else:
            self.get_logger().warn('subreaper unavailable')

        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.status_publisher = self.create_publisher(RgbCameraResolution,
            'rgb_camera_manager_server/resolution_mode', qos_profile = latching_qos)

        self.switch_resolution_srv = self.create_service(
            SwitchResolution, 'rgb_camera_manager_server/switch_resolution', self.switch_resolution)

        self.set_status(RgbCameraResolution.RESOLUTION_IDLE)

        # 服务启动时初始化启动低分辨率
        self.set_status(RgbCameraResolution.RESOLUTION_LOW)
        proc = self._spawn(LAUNCH_LOW)
        self.current_procs = [proc] if proc is not None else []
        if proc is None:
            self.set_status(RgbCameraResolution.RESOLUTION_IDLE)

        self.create_timer(REAP_INTERVAL, self._reap_exited_children)

    def _reap_exited_children(self):
        """Non-blocking reap of every exited child (direct or adopted via subreaper).

        Also syncs returncode of Popen wrappers reaped here so later
        proc.poll()/proc.wait() do not observe a stale None.
        """
        while True:
            try:
                pid, status = os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                return
            if pid == 0:
                return
            proc = self._tracked.pop(pid, None)
            if proc is not None and proc.returncode is None:
                proc.returncode = os.waitstatus_to_exitcode(status)

    def _group_has_live_members(self, pgid):
        """True iff some non-zombie process from our launch session remains.

        Zombies still make killpg(pgid, 0) succeed, so that probe cannot be
        used as liveness. Matching on (pgrp, sid) both ignores dead states and
        refuses a recycled pid that now leads an unrelated process group:
        start_new_session() makes sid == pgid == wrapper pid for the whole tree.
        """
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            fields = _read_stat_fields(int(entry))
            if fields is None:
                continue
            state, pgrp, sid = fields
            if pgrp == pgid and sid == pgid and state not in _DEAD_STATES:
                return True
        return False

    def _log_group_members(self, pgid):
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            fields = _read_stat_fields(int(entry))
            if fields is None or fields[1] != pgid:
                continue
            try:
                with open('/proc/%s/cmdline' % entry, 'rb') as handle:
                    cmd = handle.read().replace(b'\x00', b' ').decode('utf-8', 'replace')
            except OSError:
                cmd = ''
            self.get_logger().error(
                '  leftover pid=%s state=%s cmd=%r' % (entry, fields[0], cmd))

    def _spawn(self, launch_file):
        """Start a launch in its own session so the whole tree can be signalled.

        `ros2 launch` is only a wrapper: the process holding the V4L2 fd and the
        kernel-side capture buffers is the camera node underneath it. A new
        session makes the wrapper the leader of a process group containing both,
        so killpg still reaches the node after the wrapper exits and the node is
        reparented to PID 1.
        """
        try:
            proc = subprocess.Popen(
                ["ros2", "launch", "usb_cam", launch_file],
                start_new_session=True)
        except OSError as exc:
            self.get_logger().error('failed to spawn %s: %s' % (launch_file, exc))
            return None
        self._tracked[proc.pid] = proc
        return proc

    def set_status(self, newStatus):
        self.mode = newStatus

        msg = RgbCameraResolution()
        msg.resolution_mode = self.mode
        self.status_publisher.publish(msg)

        self._logger.info('rgb_camera_manager server transitioned to state %d' % newStatus)

    def switch_resolution(self, request:SwitchResolution.Request, response:SwitchResolution.Response):
        self._reap_exited_children()

        status_str = 'idle'
        if request.resolution_mode == SwitchResolution.Request.RESOLUTION_IDLE:
            status_str = 'idle'
        elif request.resolution_mode == SwitchResolution.Request.RESOLUTION_LOW:
            status_str = 'low'
        elif request.resolution_mode == SwitchResolution.Request.RESOLUTION_HIGH:
            status_str = 'high'
        else:
            status_str = 'idle'

        self.get_logger().info(f"Received a request for resolution {status_str}")

        response.resolution_mode = request.resolution_mode

        if self.mode == request.resolution_mode:
            self.get_logger().info(f"Current resolution is already {status_str}")
            response.success = True
            return response

        # Rate limit: refuse mode changes closer than MIN_SWITCH_INTERVAL_SEC.
        # Frequent switching OOMs the host (UVC/vb2); the client must retry later.
        now_mono = time.monotonic()
        if _switch_too_soon(self._last_switch_mono, now_mono):
            wait_s = MIN_SWITCH_INTERVAL_SEC - (now_mono - self._last_switch_mono)
            self.get_logger().warn(
                f"switch_resolution refused: too soon after previous switch "
                f"(wait {wait_s:.1f}s more, min interval "
                f"{MIN_SWITCH_INTERVAL_SEC:.0f}s) requested={status_str}")
            response.resolution_mode = request.resolution_mode
            response.success = False
            return response

        self.set_status(request.resolution_mode)
        self.stop_current()

        # Run both launches concurrently as separate processes.
        # IDLE is a successful stop with nothing to spawn; only a failed
        # spawn for LOW/HIGH is success=False.
        proc = None
        want_spawn = self.mode in (
            RgbCameraResolution.RESOLUTION_LOW, RgbCameraResolution.RESOLUTION_HIGH)
        if self.mode == RgbCameraResolution.RESOLUTION_LOW:
            proc = self._spawn(LAUNCH_LOW)
        elif self.mode == RgbCameraResolution.RESOLUTION_HIGH:
            proc = self._spawn(LAUNCH_HIGH)
        else:
            self._logger.info('rgb_camera_manager server transitioned to state %d' % RgbCameraResolution.RESOLUTION_IDLE)

        if proc is not None:
            self.current_procs = [proc]
            response.success = True
            self._last_switch_mono = time.monotonic()
        else:
            self.current_procs = []
            if want_spawn:
                self.set_status(RgbCameraResolution.RESOLUTION_IDLE)
                response.success = False
            else:
                # IDLE (stop-only) is still a completed switch for rate-limiting.
                response.success = True
                self._last_switch_mono = time.monotonic()
        return response

    def _signal_group(self, pgid, sig):
        """Signal a whole process group. Returns True if the group existed."""
        try:
            os.killpg(pgid, sig)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            self.get_logger().error('no permission to signal process group %d' % pgid)
            return False

    def _wait_stopped(self, proc, pgid, timeout):
        """Block until the wrapper is reaped and no live group member remains."""
        deadline = time.monotonic() + timeout
        while True:
            # Reap before probing so a just-died member cannot keep the
            # group looking alive (killpg(0) succeeds on zombies).
            self._reap_exited_children()
            wrapper_done = proc.poll() is not None
            group_alive = self._group_has_live_members(pgid)
            if wrapper_done and not group_alive:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.1)

    def _reap_popen(self, proc, timeout=1.0):
        """Make sure the Popen child is reaped; never leave it defunct."""
        if proc is None or proc.returncode is not None:
            return
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                'pid %s still not reapable after kill; leaving for next sweep' % proc.pid)

    def terminate(self, proc: subprocess.Popen, timeout: float = SHUTDOWN_TIMEOUT):
        """Stop a spawned launch and every process in its group, blocking until gone.

        Signalling the group rather than the `ros2 launch` wrapper is what
        actually reaches the camera node. The previous early return on
        `proc.poll() is not None` only observed the wrapper, so a node orphaned
        by an already-exited wrapper was never signalled at all and kept the
        camera fd and its kernel-side buffers open indefinitely.
        """
        if proc is None:
            return

        # If the wrapper was already reaped (entry sweep, prior poll), its pid
        # is free and may be recycled — never killpg a recycled pid.
        if proc.poll() is not None:
            self._reap_exited_children()
            return

        pgid = proc.pid  # start_new_session=True makes the child its own leader
        self._tracked[proc.pid] = proc

        try:
            if not self._signal_group(pgid, SIGINT) and proc.poll() is not None:
                return

            if self._wait_stopped(proc, pgid, timeout):
                return

            self.get_logger().warn(
                'process group %d did not stop within %.1fs, sending SIGKILL' % (pgid, timeout))
            # Signal only while the session still has live members. A dead
            # wrapper whose orphans remain in the same session still needs
            # SIGKILL; a fully dead group means the pid is free — never signal.
            if self._group_has_live_members(pgid) or proc.poll() is None:
                self._signal_group(pgid, SIGKILL)

            if not self._wait_stopped(proc, pgid, KILL_TIMEOUT):
                self.get_logger().error(
                    'process group %d still has live members after SIGKILL' % pgid)
                self._log_group_members(pgid)
        finally:
            # Keep the pid mapped until reaped so a later sweep can sync
            # returncode if wait() times out here.
            self._reap_popen(proc)
            self._reap_exited_children()
            if proc.returncode is not None:
                self._tracked.pop(proc.pid, None)

    def stop_current(self):
        if self.current_procs:
            for proc in self.current_procs:
                self.terminate(proc)
            self.current_procs = []

def main():
    rclpy.init()

    service = NodeManagerService()
    rclpy.spin(service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
