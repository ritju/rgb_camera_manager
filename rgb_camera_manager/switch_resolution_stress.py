#!/usr/bin/env python3
"""Standalone stress client for the SwitchResolution service.

Every --interval seconds the client asks rgb_camera_manager_server for the next
resolution in --sequence and cross-checks three independent layers:

1. service layer -- did the call answer in time, what did it echo back;
2. state layer   -- does the latched resolution_mode topic agree;
3. device layer  -- is exactly one usb_cam node alive, who holds /dev/video*,
                    and is a frame of the requested size actually published.

Layer 3 is the point. The manager answers `success` as soon as it has spawned
`ros2 launch`, so the response says nothing about whether the camera came back
at the requested geometry, nor about whether the previous node released the V4L2
fd. Run it, watch the per-cycle lines, Ctrl-C to get the summary.

Usage (after sourcing the workspace):
    ros2 run rgb_camera_manager switch_resolution_stress
    ros2 run rgb_camera_manager switch_resolution_stress --interval 5 --cycles 20
    python3 -m rgb_camera_manager.switch_resolution_stress --sequence low,high,idle
"""

import argparse
from collections import Counter, deque
import csv
import os
import signal
import sys
import threading
import time

from capella_ros_service_interfaces.msg import RgbCameraResolution
from capella_ros_service_interfaces.srv import SwitchResolution
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image

DEFAULT_SERVICE = 'rgb_camera_manager_server/switch_resolution'
DEFAULT_STATUS_TOPIC = 'rgb_camera_manager_server/resolution_mode'
DEFAULT_IMAGE_TOPIC = '/rgb_camera_back/image_raw'

MODE_NAMES = {
    SwitchResolution.Request.RESOLUTION_IDLE: 'idle',
    SwitchResolution.Request.RESOLUTION_LOW: 'low',
    SwitchResolution.Request.RESOLUTION_HIGH: 'high',
}
NAME_MODES = {name: mode for mode, name in MODE_NAMES.items()}

# Geometry the two launch files are named after; override with --expect-*.
DEFAULT_EXPECT = {'low': (640, 480), 'high': (1280, 1024)}

CAMERA_PROC_PATTERN = 'usb_cam_node_exe'
LAUNCH_PROC_PATTERN = 'ros2 launch usb_cam'

CSV_FIELDS = [
    'index', 'wall_time', 'requested', 'latency_s', 'error', 'success', 'echo',
    'status_mode', 'frames', 'dominant_size', 'fps', 'first_frame_delay_s',
    'first_expected_delay_s', 'camera_pids', 'launcher_pids', 'device_holders',
    'problems',
]


def _pids():
    """Yield every numeric PID currently visible in /proc."""
    for entry in os.listdir('/proc'):
        if entry.isdigit():
            yield int(entry)


def _read_cmdline(pid):
    """Return the command line of pid as one string ('' when unreadable)."""
    try:
        with open('/proc/%d/cmdline' % pid, 'rb') as handle:
            raw = handle.read()
    except OSError:
        return ''
    return raw.replace(b'\x00', b' ').decode('utf-8', 'replace').strip()


def _read_ppid(pid):
    """Return the parent PID of pid, or None when /proc/<pid>/stat is gone."""
    try:
        with open('/proc/%d/stat' % pid, 'rb') as handle:
            raw = handle.read().decode('utf-8', 'replace')
        # field 2 (comm) may contain spaces, so split from the last ') '.
        return int(raw.rsplit(') ', 1)[1].split()[1])
    except (OSError, IndexError, ValueError):
        return None


def _read_video_fds(pid):
    """Return (set of open /dev/video* paths, whether the fd dir was readable)."""
    devices = set()
    try:
        entries = os.listdir('/proc/%d/fd' % pid)
    except OSError:
        return devices, False
    for entry in entries:
        try:
            target = os.readlink('/proc/%d/fd/%s' % (pid, entry))
        except OSError:
            continue
        if target.startswith('/dev/video'):
            devices.add(target)
    return devices, True


def snapshot_processes():
    """Snapshot camera nodes, launch wrappers and /dev/video* holders.

    Scanning every PID (not just the usb_cam ones) is deliberate: the failure
    mode worth catching is a process from an earlier generation that outlived
    its `ros2 launch` wrapper and still owns the V4L2 fd.
    """
    camera = []
    launchers = []
    holders = {}
    unreadable = 0
    myself = os.getpid()

    for pid in _pids():
        if pid == myself:
            continue
        devices, readable = _read_video_fds(pid)
        if not readable:
            unreadable += 1
        for device in devices:
            holders.setdefault(device, []).append(pid)
        cmdline = _read_cmdline(pid)
        info = {
            'pid': pid,
            'ppid': _read_ppid(pid),
            'cmdline': cmdline,
            'devices': devices,
        }
        if CAMERA_PROC_PATTERN in cmdline:
            camera.append(info)
        elif LAUNCH_PROC_PATTERN in cmdline:
            launchers.append(info)

    return {
        'camera': camera,
        'launchers': launchers,
        'device_holders': holders,
        'unreadable_fds': unreadable,
    }


def format_sizes(sizes):
    """Format a (width, height) -> count mapping as a compact string."""
    if not sizes:
        return '-'
    return ', '.join('%dx%d x%d' % (size[0], size[1], count)
                     for size, count in sizes.most_common())


def format_holders(holders):
    """Format a /dev/video* -> [pid] mapping as a compact string."""
    if not holders:
        return '-'
    return ', '.join('%s->%s' % (device, ','.join(str(pid) for pid in pids))
                     for device, pids in sorted(holders.items()))


def parse_sequence(text):
    """Turn 'low,high' into a validated list of mode names."""
    names = [item.strip().lower() for item in text.split(',') if item.strip()]
    unknown = [name for name in names if name not in NAME_MODES]
    if unknown:
        raise argparse.ArgumentTypeError(
            'unknown resolution mode(s) %s, choose from %s'
            % (','.join(unknown), ','.join(sorted(NAME_MODES))))
    return names


def parse_size(text):
    """Turn '1280x1024' into (1280, 1024)."""
    try:
        width, height = text.lower().split('x')
        return (int(width), int(height))
    except ValueError:
        raise argparse.ArgumentTypeError('expected WxH, got %r' % text)


def parse_args(argv):
    """Parse the command line."""
    parser = argparse.ArgumentParser(
        description='Repeatedly call SwitchResolution and verify what really '
                    'happened on the service, state and device layers.')
    parser.add_argument('--interval', type=float, default=5.0,
                        help='seconds between two requests (default: 5.0)')
    parser.add_argument('--cycles', type=int, default=0,
                        help='number of requests to send, 0 = until Ctrl-C (default: 0)')
    parser.add_argument('--sequence', type=parse_sequence, default=['low', 'high'],
                        help='comma separated modes to cycle through (default: low,high)')
    parser.add_argument('--settle', type=float, default=0.0,
                        help='observation window after each request; 0 = rest of the '
                             'interval (default: 0)')
    parser.add_argument('--call-timeout', type=float, default=30.0,
                        help='seconds to wait for one service response (default: 30.0)')
    parser.add_argument('--late-threshold', type=float, default=3.0,
                        help='first frame of the requested size later than this is a '
                             'problem (default: 3.0)')
    parser.add_argument('--warmup', type=float, default=15.0,
                        help='seconds to wait for the first frame before cycle 1 '
                             '(default: 15.0)')
    parser.add_argument('--service', default=DEFAULT_SERVICE,
                        help='SwitchResolution service name (default: %(default)s)')
    parser.add_argument('--status-topic', default=DEFAULT_STATUS_TOPIC,
                        help='latched RgbCameraResolution topic (default: %(default)s)')
    parser.add_argument('--image-topic', default=DEFAULT_IMAGE_TOPIC,
                        help='sensor_msgs/Image topic used as ground truth '
                             '(default: %(default)s)')
    parser.add_argument('--expect-low', type=parse_size, default=DEFAULT_EXPECT['low'],
                        help='geometry expected in low mode (default: 640x480)')
    parser.add_argument('--expect-high', type=parse_size, default=DEFAULT_EXPECT['high'],
                        help='geometry expected in high mode (default: 1280x1024)')
    parser.add_argument('--no-size-check', action='store_true',
                        help='only count frames, do not compare geometry')
    parser.add_argument('--no-proc', action='store_true',
                        help='skip the /proc scan for camera processes and open fds')
    parser.add_argument('--csv', metavar='PATH',
                        help='also append one row per cycle to this CSV file')
    return parser.parse_args(argv)


class SwitchResolutionStress(Node):
    """Client that hammers SwitchResolution and checks what really happened."""

    def __init__(self, args):
        """Create the client, the latched status subscription and the frame probe."""
        super().__init__('switch_resolution_stress')
        self.args = args
        self.expect = {} if args.no_size_check else {
            'low': args.expect_low,
            'high': args.expect_high,
        }
        self._lock = threading.Lock()
        self._frames = deque(maxlen=8192)
        self._statuses = deque(maxlen=256)

        # usb_cam publishes image_raw/camera_info with rmw_qos_profile_sensor_data,
        # so a reliable subscription would never match and would look like a dead
        # camera.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self.create_subscription(Image, args.image_topic, self._on_image, sensor_qos)

        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RgbCameraResolution, args.status_topic,
                                 self._on_status, latched_qos)

        self.client = self.create_client(SwitchResolution, args.service)

    def _on_image(self, msg):
        """Record arrival time and geometry of every frame."""
        with self._lock:
            self._frames.append((time.monotonic(), msg.width, msg.height))

    def _on_status(self, msg):
        """Record every resolution_mode publish."""
        with self._lock:
            self._statuses.append((time.monotonic(), msg.resolution_mode))

    def frames_since(self, stamp):
        """Return the (time, width, height) frames received at or after stamp."""
        with self._lock:
            return [entry for entry in self._frames if entry[0] >= stamp]

    def statuses_since(self, stamp):
        """Return the (time, mode) publishes received at or after stamp."""
        with self._lock:
            return [entry for entry in self._statuses if entry[0] >= stamp]

    def last_status(self):
        """Return the most recently published mode name, or None."""
        with self._lock:
            entries = list(self._statuses)
        if not entries:
            return None
        return MODE_NAMES.get(entries[-1][1], str(entries[-1][1]))

    def wait_for_service(self, timeout):
        """Block until the manager answers discovery; return True when it does."""
        return self.client.wait_for_service(timeout_sec=timeout)

    def wait_for_first_frame(self, timeout):
        """Block until one frame arrives on the image topic; return True on success."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._frames:
                    return True
            time.sleep(0.1)
        return False

    def wait_for_latched_status(self, timeout):
        """Block until the latched resolution_mode sample arrives.

        The manager publishes its state with TRANSIENT_LOCAL, so a late
        subscriber always gets the current value. Draining it before cycle 1
        matters: a latched sample delivered *inside* a cycle window would
        otherwise be counted as a fresh publish and hide a missing one.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._statuses:
                    return True
            time.sleep(0.05)
        return False

    def call(self, mode):
        """Send one SwitchResolution request; return (response, latency, error)."""
        request = SwitchResolution.Request()
        request.resolution_mode = mode
        started = time.monotonic()
        future = self.client.call_async(request)
        while not future.done():
            if time.monotonic() - started > self.args.call_timeout:
                future.cancel()
                return None, time.monotonic() - started, \
                    'no response within %.1fs' % self.args.call_timeout
            time.sleep(0.01)
        latency = time.monotonic() - started
        try:
            return future.result(), latency, None
        except Exception as exc:  # noqa: B902 - report any rclpy failure verbatim
            return None, latency, '%s: %s' % (type(exc).__name__, exc)

    def run_cycle(self, index, mode_name):
        """Run one switch cycle and return its report as a dict."""
        mode = NAME_MODES[mode_name]
        expected = self.expect.get(mode_name)
        snap_before = None if self.args.no_proc else snapshot_processes()
        # The latched state tells us whether the manager is allowed to take its
        # already-in-mode early return, which legitimately publishes nothing.
        state_before = self.last_status()

        stamp = time.monotonic()
        response, latency, error = self.call(mode)

        window = self.args.settle
        if window <= 0.0:
            window = max(self.args.interval - latency - 0.2, 0.5)
        deadline = time.monotonic() + window
        while time.monotonic() < deadline:
            time.sleep(0.02)

        frames = self.frames_since(stamp)
        statuses = self.statuses_since(stamp)
        snap_after = None if self.args.no_proc else snapshot_processes()

        sizes = Counter((width, height) for _, width, height in frames)
        report = {
            'index': index,
            'wall_time': time.strftime('%H:%M:%S'),
            'requested': mode_name,
            'expected': expected,
            'latency': latency,
            'error': error,
            'success': None if response is None else bool(response.success),
            'echo': None if response is None else MODE_NAMES.get(
                response.resolution_mode, str(response.resolution_mode)),
            'window': window,
            'frames': len(frames),
            'sizes': sizes,
            'fps': len(frames) / window if window > 0 else 0.0,
            'first_frame_delay': (frames[0][0] - stamp) if frames else None,
            'first_expected_delay': None,
            'status_mode': MODE_NAMES.get(statuses[-1][1], str(statuses[-1][1]))
            if statuses else None,
            'status_publishes': len(statuses),
            'state_before': state_before,
            'noop': len(statuses) == 0 and state_before == mode_name,
            'snapshot': snap_after,
            'pids_before': {p['pid'] for p in snap_before['camera']}
            if snap_before else set(),
            'pids_after': {p['pid'] for p in snap_after['camera']}
            if snap_after else set(),
        }
        if expected:
            hits = [entry for entry in frames if (entry[1], entry[2]) == expected]
            if hits:
                report['first_expected_delay'] = hits[0][0] - stamp
        report['problems'] = self.judge(report)
        return report

    def judge(self, report):
        """Turn one cycle report into a list of concrete problem strings."""
        args = self.args
        mode_name = report['requested']
        problems = []

        if report['error']:
            problems.append('SERVICE_ERROR: %s' % report['error'])
        else:
            if report['success'] is not True:
                problems.append('RESPONSE_NOT_SUCCESS: success=%r' % (report['success'],))
            if report['echo'] != mode_name:
                problems.append('ECHO_MISMATCH: asked %s, response says %s'
                                % (mode_name, report['echo']))
        if report['latency'] > args.interval:
            problems.append('SLOW_RESPONSE: %.2fs > interval %.2fs, the blocking '
                            'callback makes requests queue up'
                            % (report['latency'], args.interval))

        if report['status_publishes'] == 0:
            if not report['noop']:
                problems.append('NO_STATUS_PUBLISH: nothing on %s, and the latched state '
                                'was %s rather than the requested %s'
                                % (args.status_topic, report['state_before'], mode_name))
        elif report['status_mode'] != mode_name:
            problems.append('STATUS_MISMATCH: asked %s, topic says %s'
                            % (mode_name, report['status_mode']))

        expected = report['expected']
        if mode_name == 'idle':
            if report['frames']:
                problems.append('STILL_STREAMING: %d frames within %.1fs after idle (%s)'
                                % (report['frames'], report['window'],
                                   format_sizes(report['sizes'])))
        elif report['frames'] == 0:
            problems.append('NO_FRAMES: nothing on %s within %.1fs'
                            % (args.image_topic, report['window']))
        elif expected:
            if report['sizes'].get(expected, 0) == 0:
                problems.append('SIZE_MISMATCH: expected %dx%d, saw %s'
                                % (expected[0], expected[1], format_sizes(report['sizes'])))
            elif report['first_expected_delay'] is not None and \
                    report['first_expected_delay'] > args.late_threshold:
                problems.append('LATE_FIRST_FRAME: %dx%d only after %.2fs (threshold %.1fs)'
                                % (expected[0], expected[1], report['first_expected_delay'],
                                   args.late_threshold))

        snap = report['snapshot']
        if snap is None:
            return problems

        want_nodes = 0 if mode_name == 'idle' else 1
        want_launchers = 0 if mode_name == 'idle' else 1
        live_pids = {proc['pid'] for proc in snap['camera']}

        if len(snap['camera']) != want_nodes:
            problems.append('CAMERA_NODE_COUNT: expected %d usb_cam node(s), found %d [%s]'
                            % (want_nodes, len(snap['camera']),
                               ','.join(str(pid) for pid in sorted(live_pids)) or '-'))
        if len(snap['launchers']) != want_launchers:
            problems.append('LAUNCHER_COUNT: expected %d `ros2 launch` wrapper(s), found %d'
                            % (want_launchers, len(snap['launchers'])))
        for proc in snap['camera']:
            if proc['ppid'] == 1:
                problems.append('ORPHANED_NODE: pid %d reparented to init, its launch '
                                'wrapper is gone' % proc['pid'])
        if want_nodes and not snap['device_holders']:
            problems.append('DEVICE_NOT_OPEN: no process holds a /dev/video* fd')
        for device, pids in sorted(snap['device_holders'].items()):
            strangers = [pid for pid in pids if pid not in live_pids]
            if strangers:
                problems.append('DEVICE_HELD_BY_STRANGER: %s still open in pid %s, which is '
                                'not a live camera node'
                                % (device, ','.join(str(pid) for pid in strangers)))
            elif len(pids) > 1:
                problems.append('DEVICE_SHARED: %s open in %d nodes at once'
                                % (device, len(pids)))
        return problems


def print_cycle(report, verbose_procs):
    """Print the one-line verdict for a cycle plus any problems below it."""
    snap = report['snapshot']
    first = report['first_frame_delay']
    first_expected = report['first_expected_delay']
    status_field = report['status_mode'] or '-'
    if report['noop']:
        status_field += '(noop)'
    sys.stdout.write(
        '[%3d] %-4s resp=%-7s lat=%6.0fms echo=%-4s status=%-9s frames=%-4d %s fps=%.1f '
        'first=%s expect=%s\n' % (
            report['index'], report['requested'],
            'ERR' if report['error'] else ('ok' if report['success'] else 'false'),
            report['latency'] * 1000.0,
            report['echo'] or '-', status_field,
            report['frames'], format_sizes(report['sizes']), report['fps'],
            '-' if first is None else '%.2fs' % first,
            '-' if first_expected is None else '%.2fs' % first_expected))
    if snap is not None and verbose_procs:
        sys.stdout.write('      pids %s -> %s  launchers=%d  %s\n' % (
            ','.join(str(pid) for pid in sorted(report['pids_before'])) or '-',
            ','.join(str(pid) for pid in sorted(report['pids_after'])) or '-',
            len(snap['launchers']), format_holders(snap['device_holders'])))
    for problem in report['problems']:
        sys.stdout.write('      !! %s\n' % problem)
    if not report['problems']:
        sys.stdout.write('      ok\n')
    sys.stdout.flush()


def print_summary(reports, elapsed):
    """Print the aggregate verdict and return the process exit code."""
    bad = [report for report in reports if report['problems']]
    print('\n=== %d cycles in %.1fs: %d ok, %d with problems ==='
          % (len(reports), elapsed, len(reports) - len(bad), len(bad)))
    if not reports:
        return 0

    latencies = [report['latency'] for report in reports]
    print('service latency   min=%.0fms avg=%.0fms max=%.0fms'
          % (min(latencies) * 1000.0,
             sum(latencies) / len(latencies) * 1000.0,
             max(latencies) * 1000.0))
    delays = [report['first_expected_delay'] for report in reports
              if report['first_expected_delay'] is not None]
    if delays:
        print('first %s frame  min=%.2fs avg=%.2fs max=%.2fs (n=%d)'
              % ('expected-size', min(delays), sum(delays) / len(delays), max(delays),
                 len(delays)))
    for mode_name in sorted({report['requested'] for report in reports}):
        subset = [report for report in reports if report['requested'] == mode_name]
        subset_bad = [report for report in subset if report['problems']]
        sizes = Counter()
        for report in subset:
            sizes.update(report['sizes'])
        print('  %-4s %3d cycles, %3d bad, sizes seen: %s'
              % (mode_name, len(subset), len(subset_bad), format_sizes(sizes)))

    counts = Counter(problem.split(':', 1)[0] for report in bad
                     for problem in report['problems'])
    if counts:
        print('problems:')
        for name, count in counts.most_common():
            print('  %-26s x%d' % (name, count))
    return 1 if bad else 0


def write_csv(path, report):
    """Append one cycle row to the CSV file, creating the header when needed."""
    snap = report['snapshot']
    dominant = report['sizes'].most_common(1)
    row = {
        'index': report['index'],
        'wall_time': report['wall_time'],
        'requested': report['requested'],
        'latency_s': '%.4f' % report['latency'],
        'error': report['error'] or '',
        'success': report['success'],
        'echo': report['echo'] or '',
        'status_mode': report['status_mode'] or '',
        'frames': report['frames'],
        'dominant_size': '%dx%d' % dominant[0][0] if dominant else '',
        'fps': '%.2f' % report['fps'],
        'first_frame_delay_s': '' if report['first_frame_delay'] is None
        else '%.3f' % report['first_frame_delay'],
        'first_expected_delay_s': '' if report['first_expected_delay'] is None
        else '%.3f' % report['first_expected_delay'],
        'camera_pids': ';'.join(str(pid) for pid in sorted(report['pids_after'])),
        'launcher_pids': '' if snap is None
        else ';'.join(str(proc['pid']) for proc in snap['launchers']),
        'device_holders': '' if snap is None else format_holders(snap['device_holders']),
        'problems': ' | '.join(report['problems']),
    }
    new_file = not os.path.exists(path)
    with open(path, 'a', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        if new_file:
            writer.writeheader()
        writer.writerow(row)


def _on_sigint(signum, frame):
    """Make Ctrl-C interrupt the main thread even if rclpy installed a handler."""
    raise KeyboardInterrupt()


def main(argv=None):
    """Run the stress loop until --cycles is reached or Ctrl-C arrives."""
    args = parse_args(sys.argv[1:] if argv is None else argv)
    signal.signal(signal.SIGINT, _on_sigint)

    rclpy.init()
    node = SwitchResolutionStress(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    reports = []
    started = time.monotonic()
    exit_code = 0
    try:
        print('service      : %s' % args.service)
        print('status topic : %s (transient_local)' % args.status_topic)
        print('image topic  : %s (sensor_data / best_effort)' % args.image_topic)
        print('sequence     : %s every %.1fs, cycles=%s'
              % (','.join(args.sequence), args.interval,
                 args.cycles if args.cycles else 'unlimited'))
        print('expected size: %s' % (', '.join(
            '%s=%dx%d' % (name, size[0], size[1])
            for name, size in sorted(node.expect.items())) or 'not checked'))

        if not node.wait_for_service(timeout=args.warmup):
            print('ERROR: service %s not available after %.1fs -- is '
                  'rgb_camera_manager running?' % (args.service, args.warmup))
            return 2
        if not node.wait_for_latched_status(5.0):
            print('WARNING: no latched sample on %s within 5.0s -- wrong topic name or '
                  'durability, already-in-mode cycles will look like missing publishes'
                  % args.status_topic)
        print('service found, current state: %s' % (node.last_status() or 'unknown'))
        if not args.no_proc:
            snap = snapshot_processes()
            print('camera nodes : %s' % (','.join(
                str(proc['pid']) for proc in snap['camera']) or '-'))
            print('video fds    : %s' % format_holders(snap['device_holders']))
            if snap['unreadable_fds']:
                print('note: %d process(es) had unreadable fd dirs, the device-holder '
                      'view may be incomplete (re-run under sudo for a full one)'
                      % snap['unreadable_fds'])
        if not node.wait_for_first_frame(args.warmup):
            print('WARNING: no frame on %s within %.1fs, starting anyway'
                  % (args.image_topic, args.warmup))
        print('')

        index = 0
        while args.cycles == 0 or index < args.cycles:
            mode_name = args.sequence[index % len(args.sequence)]
            cycle_start = time.monotonic()
            report = node.run_cycle(index + 1, mode_name)
            reports.append(report)
            print_cycle(report, verbose_procs=not args.no_proc)
            if args.csv:
                write_csv(args.csv, report)
            index += 1
            remaining = args.interval - (time.monotonic() - cycle_start)
            if remaining > 0 and (args.cycles == 0 or index < args.cycles):
                time.sleep(remaining)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        exit_code = print_summary(reports, time.monotonic() - started) or exit_code
        try:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: B902 - shutdown after Ctrl-C may already be done
            pass
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
