---
feature: switch-rate-limit
status: delivered
updated: 2026-10-23
branch: master
commits: b8ebee4..3110ccd
---

# Switch Rate Limit

## Report

**What was built** — `rgb_camera_manager` 拒绝间隔小于 10s 的分辨率切换：`switch_resolution` 在真正改 mode 前用 `_switch_too_soon()` 检查距上次成功切换的 `time.monotonic()` 间隔，过快则 `success=False` 并 WARN（含需再等秒数），不 stop/spawn。首次切换与同 mode 幂等请求不计限。`usb_cam` 不再把 `scripts/ffplay_*.sh` 装进 install（仅保留 `show_image.py`）。

**Verification** — `python3 -m pytest test/test_switch_rate_limit.py test/test_zombie_reaper.py` → 14 passed；`colcon build --packages-select usb_cam` PASS（install 列表仅 `show_image.py`）。

**Journey log** —
1. 真机对照：ffplay 多路 + &lt;2s 切换也 OOM → 归属 UVC/REQBUFS；≥15s 安全。策略常量取 10s 下限。
2. `SwitchResolution.srv` 仅有 `success`，失败原因走日志。

## [S1] Problem

频繁切换 `rgb_camera_back` 分辨率会在多路摄像头负载下触发 OOM（UVC/vb2 REQBUFS）。实测间隔 ≥15s 安全；需要在 `rgb_camera_manager` 拒绝过快切换，避免上层打爆。

## [S2] Design

- 常量 `MIN_SWITCH_INTERVAL_SEC = 10.0`。
- `_switch_too_soon(last_switch_mono, now_mono, min_interval=…)` 纯函数：`last` 为空/0 表示尚未切换，恒允许。
- `switch_resolution`：同 mode 直接 `success=True` 不记时；mode 会变化且过快 → `success=False`，**不**改 status、**不** stop/spawn、**不**更新 `_last_switch_mono`。
- 成功完成的切换（含 IDLE 停流）更新 `_last_switch_mono`。
- 失败响应的 `resolution_mode` 回显请求值（与既有行为一致）。

## [S3] Out of Scope

- 改 `SwitchResolution.srv` 增加 message 字段
- usb_cam 内再加一层间隔限制
- 诊断脚本删除（只从 install 移除）

## Tasks

- [x] T1: `_switch_too_soon` + 常量 + `switch_resolution` 拒绝路径 — acceptance: 过快返回 success=False 且不 spawn (covers: S2)
- [x] T2: `test_switch_rate_limit.py` — acceptance: 5 例绿（首切、过快、边界 10s、自定义间隔） (covers: S2)
- [x] T3: usb_cam 移除 ffplay_* 安装 — acceptance: CMake install 仅 show_image.py (covers: S2)
