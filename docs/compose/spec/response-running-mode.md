---
feature: response-running-mode
status: delivered
updated: 2026-10-23
branch: response-running-mode
commits: 0e15048..218861c
---

# Response Reports Running Mode

## Report

**What was built** — `switch_resolution` 的 `response.resolution_mode` 现在始终等于返回时刻的 `self.mode`（manager 落定状态），不再回显请求值。spawn 失败回滚路径返回 IDLE，限流拒绝路径返回切换前的旧模式；幂等命中与切换成功行为不变。`success` 语义未改。

压测工具 `switch_resolution_stress.judge()` 的 `ECHO_MISMATCH`（echo vs 请求）替换为 `RESPONSE_MODE_MISMATCH`（`response.resolution_mode` vs 实际观测态：本轮有 topic 发布取 `status_mode`，否则取 `state_before`），限流拒绝不再误报，响应与实际状态不一致仍会报警。

**Verification** — 测试先行：5 例对未修代码先红（限流 `2 != 1`、回滚 `2 != 0`，即回显请求值），修后全绿。`python3 -m unittest discover -s test -p 'test_*.py'` → 19/19 PASS（含 zombie-reaper 9 例）。`test_switch_rate_limit.py` 5/5 不受影响。独立 review 无 critical。

**Journey log** —
1. `self.mode` 在 `switch_resolution` 中途两处变化（`set_status(request)` 与回滚 `set_status(IDLE)`），故赋值必须放在每条 `return` 前而非入口 —— 入口赋值会读到过期值。
2. `judge()` 的 `echo` 报告键/CSV 列名保留原名（4 处引用），仅改比较逻辑；重命名是纯 churn。
3. 环境债（非本特性引入）：`python3 -m pytest` 因 `anyio`/`_pytest.scope` 插件冲突不可用，测试走 `unittest discover`；ament flake8/pep257 门禁同样只能经 pytest 运行，实际未强制。

## [S1] Problem

`switch_resolution` 的 `response.resolution_mode` 回显**请求值**（`request.resolution_mode`），而不是当前实际运行的分辨率。两条路径会对客户端说谎：

1. **spawn 失败回滚**：LOW/HIGH 的 `_spawn` 失败后 `set_status(IDLE)` 已把状态回滚到 IDLE，但响应仍返回请求的 HIGH/LOW —— 客户端被告知"已切到 HIGH"，而 manager 实际停在 IDLE。
2. **限流拒绝**：`_switch_too_soon` 拒绝后 `self.mode` 仍是切换前的旧模式，但响应仍回显新请求。

幂等命中与切换成功只是**碰巧**相等。`docs/compose/spec/switch-rate-limit.md` [S2] 曾明确冻结"失败响应的 `resolution_mode` 回显请求值"，本特性推翻该契约。

## [S2] Design

**响应契约（覆盖全部 5 条返回路径）**：`response.resolution_mode` 在返回时刻等于 `self.mode`（manager 的落定状态，由 `set_status()` 唯一维护），绝不是请求回显。

| 返回路径 | success | resolution_mode | self.mode |
|----------|---------|-----------------|-----------|
| 幂等命中（`self.mode == request`） | True | `self.mode`（== 请求） | 不变 |
| 限流拒绝 | False | `self.mode`（**切换前**模式） | 不变 |
| 切换成功（spawn OK） | True | 请求值 | 请求值 |
| spawn 失败回滚 | False | **IDLE**（非请求值） | IDLE |
| 纯停止 IDLE | True | IDLE | IDLE |

实现约束：赋值必须发生在 `return` 时刻（`self.mode` 在函数中途于 `set_status(request)` 与回滚 `set_status(IDLE)` 两处变化），不在入口赋值。`success` 语义不变。

**推翻既有契约**：`switch-rate-limit.md` [S2] 的"失败响应的 `resolution_mode` 回显请求值（与既有行为一致）"自本特性起作废。

**压测工具**：`switch_resolution_stress.judge()` 的 `ECHO_MISMATCH`（echo vs 请求）替换为 `RESPONSE_MODE_MISMATCH`（`response.resolution_mode` vs 实际观测态：本轮有 topic 发布则取 `status_mode`，否则取 `state_before`）。限流拒绝不再误报；"响应说 HIGH、topic 说 IDLE" 这类真 bug 仍会被抓到。`STATUS_MISMATCH` / `NO_STATUS_PUBLISH` 语义不变。

## [S3] Out of Scope

- 改 `SwitchResolution.srv`（加 message / width / height 字段）
- 查询 V4L2 实时输出几何（本特性仅承诺 mode 枚举）
- 调整 `success` 语义
- 重写已交付的 `switch-rate-limit.md` / `zombie-reaper.md`

## Tasks

- [x] T1: `switch_resolution` 三条出口均写 `response.resolution_mode = self.mode` — acceptance: 五条返回路径（含 spawn 失败回滚、限流拒绝）响应值等于 `self.mode` 而非请求值 (covers: S2)
- [x] T2: `switch_resolution_stress.judge()` 以 `RESPONSE_MODE_MISMATCH` 对照实际观测态 — acceptance: 限流拒绝不产生误报；响应与 `status_mode`/`state_before` 不一致时报警 (covers: S2)
- [x] T3: `test/test_response_running_mode.py` 五例（限流/回滚/成功/幂等/纯停止）— acceptance: 对当前代码先红（限流+回滚），修后全绿 (covers: S2)
- [x] T4: 全量本地回归 `test_switch_rate_limit.py` + `test_response_running_mode.py` — acceptance: 全部通过，无新增依赖 (covers: S2; depends: T1, T3)
