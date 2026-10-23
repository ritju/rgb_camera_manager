#!/usr/bin/env python3
"""Unit tests for SwitchResolution rate limiting — no ROS, no camera.

Contract: mode-changing switches closer than MIN_SWITCH_INTERVAL_SEC are
refused (success=False). Frequent switching OOMs the host.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rgb_camera_manager.node_manager import (  # noqa: E402
    MIN_SWITCH_INTERVAL_SEC,
    _switch_too_soon,
)


class TestSwitchRateLimit(unittest.TestCase):

    def test_min_interval_is_10s(self):
        self.assertEqual(MIN_SWITCH_INTERVAL_SEC, 10.0)

    def test_first_switch_always_allowed(self):
        self.assertFalse(_switch_too_soon(0.0, 0.0))
        self.assertFalse(_switch_too_soon(0.0, 100.0))
        self.assertFalse(_switch_too_soon(None, 1.0))

    def test_too_soon_is_refused(self):
        self.assertTrue(_switch_too_soon(100.0, 100.0 + 2.0))
        self.assertTrue(_switch_too_soon(100.0, 100.0 + 9.999))

    def test_boundary_and_after_allowed(self):
        self.assertFalse(_switch_too_soon(100.0, 100.0 + MIN_SWITCH_INTERVAL_SEC))
        self.assertFalse(_switch_too_soon(100.0, 100.0 + 15.0))

    def test_custom_interval(self):
        self.assertTrue(_switch_too_soon(10.0, 12.0, min_interval=5.0))
        self.assertFalse(_switch_too_soon(10.0, 15.0, min_interval=5.0))


if __name__ == '__main__':
    unittest.main()
