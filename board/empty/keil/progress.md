# 进度日志

## 2026-06-11

### 诊断阶段
- 用户反馈：说"扫描可用串口"后只返回端口名列表，没有详细信息
- 根因定位：MCP Server `serial-monitor` 已在 `cline_mcp_settings.json` 中注册，但 Cline 当前会话中未建立连接
- 降级方案（`GetPortNames()`）仅返回端口名，没有制造商/描述等详细信息
- 验证：pyserial 3.5 可正常工作，返回 16 个 COM 口的完整信息（制造商、描述、HWID等）

### 修复措施
1. ✅ 增强 `serial_monitor_server.py` 的 `list_serial_ports()` 函数：
   - 新增 VID/PID 提取（从 HWID 中解析）
   - 新增 serial_number、location 字段
   - 输出格式从 `COM53 (Microsoft)` 改为带分隔线的多行详细信息

2. ✅ 扩展 `.clinerules` 触发词：
   - 新增「查看串口」「有哪些串口」「查看COM口」「列出串口」「显示串口」「查看现在有那些串口」
   - 这些词也会触发 MCP serial_list 工具调用

3. ✅ MCP 配置检查（`cline_mcp_settings.json`）：
   - serial-monitor 已正确配置 → Python314 + serial_monitor_server.py
   - `disabled: false`，配置无误

### 剩余步骤
- 用户需要重启 Cline VS Code 窗口，让 MCP Server 重新建立连接

---

## 会话 12 - 程序丢失后恢复工程编译与启动流程

**日期:** 2026-07-15

### 已完成
- [x] 恢复 `app_init.c` 中 INS/motion 相关 include、队列创建和任务启动。
- [x] 恢复 `g_imuDataQueue / g_insPoseQueue / g_insCmdQueue / g_motionCmdQueue` 创建。
- [x] 恢复 `ins_task / ins_cmd_task / motion_task` 启动。
- [x] Keil `.uvprojx` 重新加入 `app_ins.c / app_ins_cmd.c / app_motion.c / port_uart_rx.c`。
- [x] Keil `.uvoptx` 同步补充文件状态，避免 GUI 状态漏文件。
- [x] 暂时撤出 `app_stack_monitor.c`，因为当前 FreeRTOS 配置不支持它使用的 `xTaskGetHandle()`，且它不是 INS/motion 核心功能。

### 编译结果
- [x] Keil clean rebuild 通过：`0 Error(s), 1 Warning(s)`。
- [x] build.log 确认 `app_ins.c / app_ins_cmd.c / app_motion.c / port_uart_rx.c` 均参与编译。
- [x] 唯一 warning 仍为既有 `LED_PORT` macro redefined。
## 2026-07-17 - NAV v1 implemented

- Added `app_nav` navigation layer above INS pose and motion PID.
- Commands now include `nav help/status/stop/goto/square`.
- NAV v1 uses turn-to-target, drive-to-target, turn-to-final-yaw.
- Flash logging continues to use existing INS logs.
- Keil rebuild passed with `0 Error(s), 1 Warning(s)` after cleanup; remaining warning is the existing `LED_PORT` macro redefinition.
## 2026-07-17 - Fixed NAV square sequencing

- Added `cmd_id` handshake between NAV and motion.
- NAV now waits for exact motion accept/done feedback.
- `nav square <side_m>` now executes four forward legs and four `+90 deg` turns.
- Keil rebuild passed with `0 Error(s), 1 Warning(s)`.
## 2026-07-17 - Test1 fixed route added

- Added independent `app_test1` module with `APP_TEST1_ENABLE`.
- Added UART commands: `test1 help/start/status/stop` and `Test1` alias.
- Route: `(0.90,0.00) -> (0.96,-0.90) -> (0.04,-0.96) -> (0.00,0.00)`, with right turns after each point.
- Keil rebuild passed with `0 Error(s), 1 Warning(s)`.

## 2026-07-17 - Test1/nav turn-after-drive diagnostics

- Added `[MOTION_ACK] accept/reject/done` hard echoes in `app_motion.c`.
- Added `[TEST1_STEP]` logs for drive done, send right turn, and right turn done.
- Added `[NAV_STEP]` logs for square drive done, send square turn, and square turn done.
- Keil rebuild passed with `0 Error(s), 1 Warning(s)`.

## 2026-07-17 - Arc/path v1

- Added standard arc command: `motion arc <radius_m> <angle_deg>`.
- Arc v1 uses left/right wheel-speed PID only; IMU yaw is the stop condition.
- VOFA JustFloat arc output uses 6 float channels when `LOG_PRINT_PID_ENABLE=1`.
- Added `app_path` RAM teach/replay module: `path record start/stop`, `path print`, `path replay`, `path clear`, `path status`, `path stop`.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`; `app_path.c` compiled.

## 2026-07-17 - TFT PID menu updated for arc tuning

- Updated `PID` menu entries to `Straight`, `TurnYaw`, `WheelSpd`, `Arc`, and `Yaw Status`.
- Added `PID -> Arc -> WheelSpd` so arc v1 tunes the same left/right wheel-speed PID used by the inner speed loops.
- Added `PID -> Arc -> Arc Status` live page showing state, target/actual left mps, target/actual right mps, and target/actual yaw.
- Added arc runtime status fields to `Motion_RuntimeStatus_t` for TFT display.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`; the remaining warning is the existing `LED_PORT` macro redefinition.

## 2026-07-17 - Arc weak closed-loop mode

- Changed arc wheel-speed control from strong direct PID correction to weak closed-loop correction for noisy/non-hardware encoder feedback.
- Arc actual wheel speeds are low-pass filtered before PID use.
- Arc PID trim is capped to +/-4 PWM percentage points regardless of menu `PwmTrim`.
- Arc PWM output is slew-limited to 2 percentage points per 10ms update.
- Default wheel-speed PID is now `Kp=80, Ki=0, Kd=0, PwmTrim=10`.
- VOFA arc channels remain 8 floats: target/actual left, target/actual right, target/actual yaw, left/right PWM.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`.

## 2026-07-17 - Arc left-turn asymmetry compensation

- Based on repeated tests, right arc `motion arc 0.50 -90` was stable around `X=0.59, Y=-0.53`, while left arc `motion arc 0.50 90` landed around `X=0.34..0.43, Y=0.39..0.42`.
- Added left-arc-only compensation: left inner wheel target scale `1.10`, right outer wheel target scale `0.95`.
- Right arc target ratio is unchanged.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`.

## 2026-07-17 - Path replay changed to recorded arc segments

- `path replay` no longer primarily turns toward each recorded point and drives straight to it.
- Replay now uses the delta between consecutive recorded points: if yaw delta is at least 5deg and the inferred radius is 0.15..2.00m, it sends `motion arc <radius> <yaw_delta>`.
- Near-straight recorded segments still use `motion fwd <distance>`.
- This better matches the user priority: push a curve once, then have the car drive the pushed curve.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`.

## 2026-07-17 - Path replay continuous tracking v2

- Implemented `PATH_STATE_REPLAY_TRACK` as the default `path replay` mode.
- Replay now continuously reads INS pose, selects a lookahead point, computes heading error, and directly updates left/right TB6612 PWM every 50ms.
- Defaults: lookahead `0.12m`, base PWM `16`, yaw Kp `0.25`, trim limit `+/-8`, PWM range `6..30`, slew `2` per update.
- `path status` now reports target index, tracking distance, heading error, and current left/right PWM.
- Added throttled `[PATH_TRACK]` logs every 500ms.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`.

## 2026-07-17 - Path tracking curvature tighten

- Continuous replay removed stop-and-go and polygonal behavior, but replay arc radius was slightly too large.
- Tuned tracking constants: lookahead `0.12m -> 0.10m`, yaw Kp `0.25 -> 0.35`, completion distance `0.08m -> 0.06m`.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`.

## 2026-07-17 - Path record capacity increased

- Increased RAM path capacity from 64 points to 160 points, so `path record` can collect at least 150 points.
- Changed path count and replay target indexes from 8-bit to 16-bit to avoid overflow above 255.
- Memory impact is modest: each path point is 12 bytes, so 160 points use 1920 bytes.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`; RAM summary was `RW-data=192`, `ZI-data=30984`.
