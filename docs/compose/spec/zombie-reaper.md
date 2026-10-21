---
feature: zombie-reaper
status: delivered
updated: 2026-10-21
branch: feature/zombie-reaper
commits: bbc7710..2eea05b
---

# Zombie Reaper

## Report

**What was built** — Dual-layer zombie defense for the SwitchResolution lifecycle. `node_manager.py` now judges process-group liveness from `/proc` (ignoring `Z`/`X`/`x`) instead of `killpg(0)`, which is zombie-blind and caused every switch to burn 10s+10s before abandoning unreaped children. Every `terminate()` ends in a guaranteed `Popen.wait()`, a `waitpid(-1, WNOHANG)` sweep reaps direct and subreaper-adopted orphans (with pid→Popen returncode sync), and a 1s idle timer sweeps strays between calls. `PR_SET_CHILD_SUBREAPER` adopts `usb_cam_node` grandchildren when the wrapper dies first. Spawn `OSError` reports `success=False` and rolls the latched state to IDLE; an IDLE request remains `success=True`. Both `rgb_camera_back_*.launch.py` drop `respawn=True` so launch no longer races the manager's SIGINT/SIGKILL (and no longer collides with `respawn_delay == SHUTDOWN_TIMEOUT`).

**Verification** — From `rgb_camera_manager/.worktrees/zombie-reaper`:
- `python3 -m pytest test/test_zombie_reaper.py` — **PASS 9/9** (C1 liveness pin, C2 sweep + subreaper adoption, C3 real `terminate()` reap + no-signal-on-reaped, C6 IDLE success shape)
- 10-call service hammer (low/high/**idle** ×2) — **PASS**: all `success=True`, max tree-zombies=0, max latency 0.105s (old path could block 10–20s)
- `switch_resolution_stress --interval 3 --cycles 5` — service lat=10ms OK; `NO_FRAMES`/`CAMERA_NODE_COUNT`/`DEVICE_NOT_OPEN` are **ENV** (no `/dev/video*` in this sandbox), not code defects. post-stop defunct under manager: 0
- `flake8 rgb_camera_manager/node_manager.py` — **PRE-EXISTING** E501/E128 style (baseline 30, now 32; not introduced by this change)
- Reviewer (fresh subagent): 2 CRITICAL (IDLE `success=False` regression; pgid reuse) — fixed; focused re-review **PASS**

**Journey log** —
1. `killpg(pgid, 0)` succeeds on zombies — any "process group alive" probe built on it is zombie-blind. `/proc/<pid>/stat` state must be filtered explicitly.
2. CPython `Popen.poll()` sets `returncode=0` on `ECHILD`; an external `waitpid(-1)` sweep must sync `proc.returncode` via a pid→Popen map or exit codes silently lie.
3. First pgid-reuse guard (skip SIGKILL when wrapper already dead) was too aggressive — orphaned session-mates that ignore SIGINT need SIGKILL. Fix: signal while `_group_has_live_members(pgid)` (match `pgrp==sid==pgid`), skip only when the session is fully dead.
4. Default stress sequence `low,high` skipped IDLE and masked the success=False regression. Boundary values belong in every verification sequence.
5. Env notes: this sandbox has no `/dev/video*`; `/map/config/usb_cam/*.yaml` was created for bring-up; ros2 launch needs `pydantic<2` on this Humble install.

## [S1] Problem

频繁调用 `SwitchResolution` 切换分辨率后，系统出现大量 `<defunct>` 僵尸进程，占用进程表项。manager 直接子进程（`ros2 launch` 包装器）与 launch 子树内孙进程（`usb_cam_node_exe`）两种都可能残留。

根因：

| ID | 根因 | 证据 |
|---|---|---|
| H1 | `_wait_stopped` 用 `os.killpg(pgid, 0)` 探活。僵尸仍占进程组，`killpg(…, 0)` 对僵尸返回成功，探活永真 → 每次切换空等 10s+10s；超时后 Popen 被丢弃 | `node_manager.py` `_wait_stopped` |
| H2 | 无全局子进程收割。`terminate` 超时路径无最终 `wait()`；子进程在两次切换之间自行崩溃则一直 defunct | `terminate` |
| H3 | `killpg(SIGKILL)` 时 `ros2 launch` 先死；孙进程若 PID 1 不回收（容器）则永久 defunct | 无 subreaper |
| H4 | launch 文件 `respawn=True, respawn_delay=10` 与 manager 的 SIGINT/SIGKILL 竞态；`respawn_delay == SHUTDOWN_TIMEOUT` 加重时序冲突。sizeimage-guard H2 已记录 respawn 是故障放大器 | `rgb_camera_back_*.launch.py` |
| H5 | 外部 `waitpid(-1)` 抢先收割时，CPython `Popen` 会把 `returncode` 静默记成 0 | 需 pid→Popen 映射 |

## [S2] Design

范围：加固 `rgb_camera_manager` 的进程终止/收割；两个 `rgb_camera_back_*.launch.py` 关闭 respawn。不改服务 API，不直拉 `usb_cam_node_exe`，不引 psutil。

### C1 僵尸感知进程组探活

`_group_has_live_members(pgid)` 扫 `/proc/*/stat`，仅当 `state not in ('Z','X','x')` **且** `pgrp == sid == pgid` 才算存活（session 匹配拒绝 pid 复用后的无关进程组）。`_wait_stopped` 改用此判定，不再 `killpg(pgid, 0)`。

可观测：干净退出后 `_wait_stopped` 在 ~1s 内返回 True，不再出现 "did not stop within 10.0s"。

### C2 全局子进程收割

`_reap_exited_children()` 循环 `os.waitpid(-1, os.WNOHANG)` 至 `ChildProcessError` / `pid==0`。维护 `self._tracked: dict[pid, Popen]`：外部收割到已跟踪 pid 时同步 `proc.returncode = os.waitstatus_to_exitcode(status)`；`terminate` 仅在 `returncode is not None` 后 pop，超时则留给下一轮 sweep 同步。

调用点：`_wait_stopped` 每轮（探活前）、`switch_resolution` 入口、`terminate` 的 finally。

可观测：切换间 `kill -9 <ros2-launch-pid>` 后 ≤1.5s 内无 defunct 子进程；同模式 hammer 也保持 0 defunct。

### C3 保证收割 Popen wrapper

`terminate` 入口若 `proc.poll() is not None`（已收割，pid 可能复用）直接清扫返回，**不发信号**。否则 `finally` 必调 `_reap_popen(proc)`（短 `proc.wait(timeout=1.0)`）。SIGKILL 后等待用 `KILL_TIMEOUT=2.0`。SIGKILL 仅在 `_group_has_live_members(pgid) or proc.poll() is None` 时发出：wrapper 已死但同 session 孤儿仍在时必须补刀；session 已空则绝不 `killpg` 可能复用的 pid。

可观测：`terminate` 返回后 `proc.returncode is not None`；已收割的 wrapper 不会触发 killpg。

### C4 Child Subreaper

`ctypes` 调 `prctl(PR_SET_CHILD_SUBREAPER=36, 1)`。成功/失败打日志（失败含 `subreaper unavailable`），失败降级不阻塞。配合 C2，`usb_cam_node` 孤儿被 manager 收养并在退出后收割。

可观测：SIGKILL wrapper 后，`usb_cam_node` 的 ppid == manager pid 而非 1；其退出后一个清扫周期内消失。prctl 失败时日志含 `subreaper unavailable` 且 C1–C3 仍成立。

### C5 空闲定时清扫

`create_timer(REAP_INTERVAL=1.0, self._reap_exited_children)`。单线程 executor 在 `terminate` 阻塞期间 timer 不触发是预期行为，故 C2 同时内联在等待环内。

### C6 启动失败与 IDLE 语义

`_spawn` 包 `try/except OSError`，失败返回 `None`。LOW/HIGH spawn 失败 → `response.success = False` 且 `set_status(RESOLUTION_IDLE)`。**IDLE 请求是成功的停止，`success=True`**（无进程可拉起）。启动时 spawn 失败则 `set_status(RESOLUTION_IDLE)`。保留 `start_new_session=True`。

可观测：`ros2` 不在 PATH 时服务不抛异常，LOW/HIGH 的 `success=False`；IDLE 切换 `success=True`。

### C7 同模式 early-return 仍清扫

`switch_resolution` 入口先 `_reap_exited_children()`，再走原 `self.mode == request.resolution_mode` 分支。

### 常量

`SHUTDOWN_TIMEOUT=10.0`（不变）、`KILL_TIMEOUT=2.0`、`REAP_INTERVAL=1.0`、`PR_SET_CHILD_SUBREAPER=36`。

### 不装 SIGCHLD handler

rclpy 持有信号处理；Python 信号 handler 仅在主线程字节码间隙运行，与 `Popen.wait()` 竞态。`waitpid(-1, WNOHANG)` + timer 足够。

### launch 文件

两个 `rgb_camera_back_*.launch.py` 的 `Node(...)` 块：删除 `respawn_delay=10`，显式 `respawn=False` 并注释意图。

## [S3] Out of Scope

- 绕过 `ros2 launch` 直拉 `usb_cam_node_exe`
- 重新引入 psutil
- 服务 API / topic / QoS / latched `resolution_mode` 语义变更
- manager 自身退出时的 `stop_current()`（可选后续）
- `usb_cam/launch/camera.launch.py`、usb_cam C++ / sizeimage-guard 代码
- 多线程 executor / 异步 service 回调
- wrapper 在 `terminate` 入口前已被收割时的孤儿补刀（pid 已复用风险大于收益；由 C2/C4 清扫兜底）

## Tasks

- [x] T1: worktree ×2 + stress 工具入仓 — acceptance: 两仓可 build；`switch_resolution_stress --help` 可用
- [x] T2: C1+C2+C3+C6+C7（终止/收割加固）— acceptance: `terminate` 返回后 `returncode is not None`；切换无 10s 探活误报 (covers: S2)
- [x] T3: C4+C5（subreaper + timer）— acceptance: 探针显示孤儿 ppid==manager；prctl 失败降级有日志 (covers: S2; depends: T2)
- [x] T4: 两 launch 文件 respawn=False — acceptance: `kill -9 usb_cam_node` 后 15s 不复活；切换可恢复 (covers: S2)
- [x] T5: Verify 全套（stress 三档 + ps + 边界探针）— acceptance: 无 problems、0 defunct、latency 改善 (covers: S2; depends: T2, T3, T4)
- [x] T6: Review + 修 critical — acceptance: reviewer 三项结论通过 (covers: S2; depends: T5)
- [x] T7: Spec finalize + 提交 — acceptance: 文档 delivered，commits 范围正确 (covers: S2; depends: T6)
