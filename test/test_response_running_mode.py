#!/usr/bin/env python3
"""Regression tests: SwitchResolution response reports the running mode.

Contract: response.resolution_mode is always self.mode at return time, not the
request echo — including rate-limit refuse and spawn-failure rollback.
No ROS graph, no camera, no subprocesses.
"""
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rgb_camera_manager.node_manager import NodeManagerService  # noqa: E402
from capella_ros_service_interfaces.msg import RgbCameraResolution  # noqa: E402

IDLE = RgbCameraResolution.RESOLUTION_IDLE
LOW = RgbCameraResolution.RESOLUTION_LOW
HIGH = RgbCameraResolution.RESOLUTION_HIGH


class _Logger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass

    warning = warn


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Req:
    def __init__(self, mode):
        self.resolution_mode = mode


class _Resp:
    def __init__(self):
        self.resolution_mode = None
        self.success = None


def make_manager_shell():
    """NodeManagerService with real switch_resolution, no ROS node."""
    mgr = NodeManagerService.__new__(NodeManagerService)
    mgr.current_procs = []
    mgr._tracked = {}
    mgr.get_logger = lambda: _Logger()
    mgr._logger = _Logger()
    mgr.status_publisher = _Pub()
    mgr.mode = IDLE
    mgr._last_switch_mono = 0.0
    # switch_resolution() calls this first; real impl does waitpid(-1) which
    # would steal children from the test runner. Stub it out.
    mgr._reap_exited_children = lambda: None
    return mgr


class TestResponseRunningMode(unittest.TestCase):

    def test_rate_limit_refuse_returns_previous_mode(self):
        mgr = make_manager_shell()
        mgr.set_status(LOW)
        mgr._last_switch_mono = time.monotonic()  # just switched -> refuse next
        resp = mgr.switch_resolution(_Req(HIGH), _Resp())
        self.assertIs(resp.success, False)
        self.assertEqual(resp.resolution_mode, LOW)   # NOT HIGH
        self.assertEqual(mgr.mode, LOW)               # state untouched

    def test_spawn_failure_rollback_returns_idle(self):
        mgr = make_manager_shell()
        mgr._spawn = lambda launch_file: None         # force spawn failure
        resp = mgr.switch_resolution(_Req(HIGH), _Resp())
        self.assertIs(resp.success, False)
        self.assertEqual(resp.resolution_mode, IDLE)  # NOT HIGH
        self.assertEqual(mgr.mode, IDLE)

    def test_successful_switch_returns_new_mode(self):
        mgr = make_manager_shell()
        mgr._spawn = lambda launch_file: object()     # non-None = success
        resp = mgr.switch_resolution(_Req(HIGH), _Resp())
        self.assertIs(resp.success, True)
        self.assertEqual(resp.resolution_mode, HIGH)
        self.assertEqual(mgr.mode, HIGH)

    def test_idempotent_hit_returns_current_mode(self):
        mgr = make_manager_shell()
        mgr.set_status(LOW)
        resp = mgr.switch_resolution(_Req(LOW), _Resp())
        self.assertIs(resp.success, True)
        self.assertEqual(resp.resolution_mode, LOW)
        self.assertEqual(mgr.mode, LOW)

    def test_stop_only_idle_returns_idle(self):
        mgr = make_manager_shell()
        mgr.set_status(LOW)

        def _must_not_spawn(launch_file):
            raise AssertionError('_spawn must not be consulted for IDLE')

        mgr._spawn = _must_not_spawn
        resp = mgr.switch_resolution(_Req(IDLE), _Resp())
        self.assertIs(resp.success, True)
        self.assertEqual(resp.resolution_mode, IDLE)
        self.assertEqual(mgr.mode, IDLE)


if __name__ == '__main__':
    unittest.main(verbosity=2)
