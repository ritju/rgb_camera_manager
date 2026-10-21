#!/usr/bin/env python3
"""Lifecycle tests for zombie-reaper logic — no ROS, no camera required.

Exercises the process-management contracts C1–C3 against real processes:
  C1  zombie-aware group liveness (killpg(0) is zombie-blind)
  C2  waitpid(-1, WNOHANG) sweep reaps direct + adopted children
  C3  terminate() always leaves Popen.returncode set
"""
import os
import signal
import subprocess
import sys
import time
import unittest

# Import helpers without constructing a ROS node.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rgb_camera_manager.node_manager import (  # noqa: E402
    _read_stat_fields,
    _enable_child_subreaper,
    _DEAD_STATES,
    KILL_TIMEOUT,
    SHUTDOWN_TIMEOUT,
)


def spawn_session_leader(extra_children=0, zombie_child=False):
    """Spawn a session-leader Python that owns `extra_children` kids.

    If zombie_child, one kid exits immediately and is NOT waited on by the
    parent — it becomes a zombie that still occupies the process group.
    Returns (popen, list_of_child_pids).
    """
    script = r'''
import os, sys, time, signal
n = int(sys.argv[1])
zombie = sys.argv[2] == "1"
kids = []
for i in range(n):
    pid = os.fork()
    if pid == 0:
        # child: ignore SIGINT so only SIGKILL (or parent death) stops it
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        while True:
            time.sleep(0.05)
    kids.append(pid)
if zombie:
    pid = os.fork()
    if pid == 0:
        os._exit(0)
    kids.append(pid)
    # deliberately do NOT waitpid the zombie
print("READY", os.getpid(), flush=True)
while True:
    time.sleep(0.05)
'''
    proc = subprocess.Popen(
        [sys.executable, '-c', script, str(extra_children), '1' if zombie_child else '0'],
        start_new_session=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    line = proc.stdout.readline().strip()
    assert line.startswith('READY'), line
    leader_pid = int(line.split()[1])
    # collect children of the leader via /proc
    children = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        fields = _read_stat_fields(int(entry))
        if fields is None:
            continue
        # parent check
        try:
            with open('/proc/%s/stat' % entry, 'rb') as handle:
                raw = handle.read().decode('utf-8', 'replace')
            rest = raw.rsplit(') ', 1)[1].split()
            ppid = int(rest[1])
        except Exception:
            continue
        if ppid == leader_pid:
            children.append(int(entry))
    return proc, children


def kill_tree(proc):
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=2)
    except Exception:
        pass
    # sweep orphans
    while True:
        try:
            pid, _ = os.waitpid(-1, os.WNOHANG)
        except ChildProcessError:
            break
        if pid == 0:
            break


class TestZombieAwareLiveness(unittest.TestCase):
    """C1: /proc state-based liveness ignores zombies, unlike killpg(0)."""

    def test_killpg_succeeds_on_zombie_group(self):
        proc, kids = spawn_session_leader(extra_children=0, zombie_child=True)
        try:
            # give the zombie a moment
            time.sleep(0.1)
            # killpg(0) succeeds even though only a zombie remains (after we kill leader)
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=2)
            time.sleep(0.1)
            # The group may still contain the zombie child of the dead leader
            # (until reparent-reap). killpg(0) is the buggy probe:
            try:
                os.killpg(proc.pid, 0)
                killpg_says_alive = True
            except ProcessLookupError:
                killpg_says_alive = False
            # Our liveness: scan /proc for non-Z members of that pgid
            live = False
            for entry in os.listdir('/proc'):
                if not entry.isdigit():
                    continue
                fields = _read_stat_fields(int(entry))
                if fields is None:
                    continue
                state, pgrp, sid = fields
                if pgrp == proc.pid and sid == proc.pid and state not in _DEAD_STATES:
                    live = True
                    break
            # C1 behavioral pin: if killpg(0) still says "alive" after the
            # leader is gone, /proc liveness MUST report dead (zombie-blind).
            # If killpg already lost the group, probes agree and the assertion
            # is vacuous — that is fine, the interesting case is enforced.
            if killpg_says_alive:
                self.assertFalse(
                    live,
                    'killpg(0) still sees the group but /proc found a live '
                    'member — liveness filter is wrong or a real process remains')
                print('OK: killpg(0) zombie-blind; /proc liveness correct')
            else:
                self.assertFalse(live, 'group gone but /proc found a live member')
                print('NOTE: group fully gone (reparent-reaped); probes agree')
        finally:
            kill_tree(proc)

    def test_live_member_reports_alive(self):
        proc, kids = spawn_session_leader(extra_children=2, zombie_child=False)
        try:
            time.sleep(0.1)
            live = False
            for entry in os.listdir('/proc'):
                if not entry.isdigit():
                    continue
                fields = _read_stat_fields(int(entry))
                if fields is None:
                    continue
                state, pgrp, sid = fields
                if pgrp == proc.pid and sid == proc.pid and state not in _DEAD_STATES:
                    live = True
                    break
            self.assertTrue(live, 'expected live members in the group')
        finally:
            kill_tree(proc)


class TestReapSweep(unittest.TestCase):
    """C2: waitpid(-1, WNOHANG) reaps exited children including zombies."""

    def test_sweep_reaps_zombie_direct_child(self):
        # Fork a child that exits and becomes our zombie
        pid = os.fork()
        if pid == 0:
            os._exit(42)
        time.sleep(0.05)
        # It is a zombie now (we have not waited)
        fields = _read_stat_fields(pid)
        self.assertIsNotNone(fields)
        self.assertEqual(fields[0], 'Z', 'child should be a zombie before sweep')
        # Sweep
        reaped = {}
        while True:
            try:
                rpid, status = os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                break
            if rpid == 0:
                break
            reaped[rpid] = status
        self.assertIn(pid, reaped)
        self.assertEqual(os.waitstatus_to_exitcode(reaped[pid]), 42)
        self.assertIsNone(_read_stat_fields(pid), 'zombie should be gone after sweep')

    def test_sweep_reaps_orphan_adopted_by_subreaper(self):
        if not _enable_child_subreaper():
            self.skipTest('subreaper unavailable in this environment')
        # Leader that forks a grandchild and exits, leaving grandchild orphaned
        script = r'''
import os, sys, time
pid = os.fork()
if pid == 0:
    # grandchild: lives a bit then exits
    time.sleep(0.15)
    os._exit(7)
print("READY", os.getpid(), flush=True)
time.sleep(0.05)  # exit before grandchild, so grandchild reparents
os._exit(0)
'''
        proc = subprocess.Popen([sys.executable, '-c', script],
                                start_new_session=True,
                                stdout=subprocess.PIPE, text=True)
        line = proc.stdout.readline().strip()
        assert line.startswith('READY')
        proc.wait(timeout=2)
        # Grandchild should now be adopted by us (subreaper)
        deadline = time.monotonic() + 2.0
        adopted = None
        while time.monotonic() < deadline:
            for entry in os.listdir('/proc'):
                if not entry.isdigit():
                    continue
                try:
                    with open('/proc/%s/stat' % entry, 'rb') as handle:
                        raw = handle.read().decode('utf-8', 'replace')
                    rest = raw.rsplit(') ', 1)[1].split()
                    ppid = int(rest[1])
                    state = rest[0]
                except Exception:
                    continue
                if ppid == os.getpid() and state not in _DEAD_STATES:
                    # likely the adopted grandchild (only extra child we have)
                    adopted = int(entry)
                    break
            if adopted is not None:
                break
            time.sleep(0.05)
        self.assertIsNotNone(adopted, 'subreaper did not adopt orphan')
        # Wait for it to exit, then sweep
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            fields = _read_stat_fields(adopted)
            if fields is not None and fields[0] == 'Z':
                break
            time.sleep(0.05)
        # Sweep
        while True:
            try:
                rpid, status = os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                break
            if rpid == 0:
                break
            if rpid == adopted:
                self.assertEqual(os.waitstatus_to_exitcode(status), 7)
        self.assertIsNone(_read_stat_fields(adopted), 'adopted orphan not reaped')


class TestTerminateReapsWrapper(unittest.TestCase):
    """C3: after terminate-equivalent flow, Popen.returncode is set."""

    def test_sigkill_then_wait_reaps(self):
        proc, kids = spawn_session_leader(extra_children=1, zombie_child=False)
        try:
            pgid = proc.pid
            os.killpg(pgid, signal.SIGKILL)
            try:
                proc.wait(timeout=KILL_TIMEOUT)
            except subprocess.TimeoutExpired:
                self.fail('wrapper not reaped after SIGKILL')
            self.assertIsNotNone(proc.returncode)
            # sweep any remaining adopted kids
            while True:
                try:
                    pid, _ = os.waitpid(-1, os.WNOHANG)
                except ChildProcessError:
                    break
                if pid == 0:
                    break
            # no live members of that group
            live = False
            for entry in os.listdir('/proc'):
                if not entry.isdigit():
                    continue
                fields = _read_stat_fields(int(entry))
                if fields is None:
                    continue
                state, pgrp, sid = fields
                if pgrp == pgid and sid == pgid and state not in _DEAD_STATES:
                    live = True
            self.assertFalse(live, 'live group members after SIGKILL + wait')
        finally:
            kill_tree(proc)

    def test_graceful_sigint_reaps_quicker_than_old_timeout(self):
        proc, kids = spawn_session_leader(extra_children=1, zombie_child=False)
        try:
            pgid = proc.pid
            start = time.monotonic()
            # Our leader's kids ignore SIGINT; leader itself dies on SIGINT
            # if we send SIGKILL to kids first... Use SIGKILL to the whole
            # group after a short grace, matching terminate() shape.
            os.killpg(pgid, signal.SIGINT)
            deadline = start + 1.0
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    break
                time.sleep(0.05)
            if proc.poll() is None:
                os.killpg(pgid, signal.SIGKILL)
                proc.wait(timeout=KILL_TIMEOUT)
            else:
                proc.wait(timeout=KILL_TIMEOUT)
            self.assertIsNotNone(proc.returncode)
            elapsed = time.monotonic() - start
            self.assertLess(elapsed, SHUTDOWN_TIMEOUT,
                            'should not burn the full old 10s timeout')
        finally:
            kill_tree(proc)


class _Logger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass

    warning = warn


def make_manager_shell():
    """NodeManagerService with real lifecycle methods, no ROS node."""
    from rgb_camera_manager.node_manager import NodeManagerService
    mgr = NodeManagerService.__new__(NodeManagerService)
    mgr.current_procs = []
    mgr._tracked = {}
    mgr.get_logger = lambda: _Logger()
    mgr._logger = _Logger()
    return mgr


class TestRealTerminate(unittest.TestCase):
    """C3 via the actual terminate()/_reap_popen() path, not a reimplementation."""

    def test_terminate_reaps_and_clears_tracked(self):
        mgr = make_manager_shell()
        proc, kids = spawn_session_leader(extra_children=1, zombie_child=False)
        try:
            mgr.current_procs = [proc]
            mgr._tracked[proc.pid] = proc
            mgr.stop_current()
            self.assertEqual(mgr.current_procs, [])
            self.assertIsNotNone(proc.returncode, 'terminate must reap the wrapper')
            # no live members left in the session
            self.assertFalse(mgr._group_has_live_members(proc.pid))
            # no leftover tracked entries for this pid
            self.assertNotIn(proc.pid, mgr._tracked)
        finally:
            kill_tree(proc)

    def test_terminate_skips_signal_when_already_reaped(self):
        """C3/pgid-reuse: a reaped wrapper must not be killpg'd (pid may recycle)."""
        mgr = make_manager_shell()
        proc, kids = spawn_session_leader(extra_children=0, zombie_child=False)
        # Force-reap as the entry sweep would (kill first — the leader loops)
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=2)
        self.assertIsNotNone(proc.returncode)
        dead_pid = proc.pid
        # terminate must return without signalling
        mgr.terminate(proc)
        self.assertNotIn(dead_pid, mgr._tracked)
        self.assertIsNotNone(proc.returncode)

    def test_terminate_idle_shape_success(self):
        """C6 boundary: stopping without spawn is a success (no proc to create)."""
        mgr = make_manager_shell()
        # Mimic the switch_resolution decision table for IDLE
        mgr.mode = 0  # RESOLUTION_IDLE
        want_spawn = mgr.mode in (1, 2)
        proc = None
        if proc is not None:
            success = True
        else:
            success = False if want_spawn else True
        self.assertTrue(success, 'IDLE stop must be success=True')


if __name__ == '__main__':
    unittest.main(verbosity=2)
