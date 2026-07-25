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

## 2026-07-18 - Path replay long-route early-finish guard

- Fixed continuous path replay completing early when the vehicle passes near the final recorded point before tracking most of a long route.
- Completion now requires both `final_dist <= 0.06m` and replay index within the last 3 recorded points.
- `path status` now includes `final=<m>` so final-point proximity is visible during replay.
- Added throttled `[PATH_TRACK] near final ignored ...` logs when final point proximity is ignored because the replay index is still too early.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`.

## 2026-07-18 - Path capacity 500 and stack trim

- Increased `PATH_MAX_POINTS` to 500, so path RAM storage is about 6000 bytes.
- Kept FreeRTOS heap at 18KB; reduced startup main stack to `0x0800` to pay for the extra path RAM.
- Expanded path Flash storage at `0x000F0000` from one 4KB sector to two sectors (8KB total).
- Trimmed low-usage task stacks based on observed high-water marks; kept TFT unchanged due to missing stack data.
- Added heap free/min-ever output to stack monitor.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`.

## 2026-07-18 - Gray sensor channel convention

- 8-channel grayscale line sensor convention: the leftmost physical sensor is channel 1.
- Current mux wiring is AD0=PA8, AD1=PB5, AD2=PA9, OUT=PB4.
- Gray UART logs print every active channel in the sampled frame, for example `active_ch=4,5`.

## 2026-07-18 - GrayLine PID v1

- Added gray line-following control: gray position outer PID feeding left/right wheel-speed inner PID.
- Channel weights are `-7,-5,-3,-1,+1,+3,+5,+7`; target position is 0.
- UART commands are `grayline help/status/start/stop/pid`; motors remain stopped until `grayline start`.
- TFT `PID -> GrayLine` menu edits Kp/Ki/Kd/TurnMax/BaseMps/LostMs/Slew offline.
- When `LOG_PRINT_PID_ENABLE=1`, running GrayLine emits 10-channel VOFA JustFloat data.

## 2026-07-18 - GrayLine no-speed-drop turn authority

- User constraint: do not reduce `BaseMps` while improving large-arc line following.
- Added signed left/right TB6612 speed commands so GrayLine can brake or briefly reverse the inner wheel without changing the center/base speed target.
- GrayLine target wheel speeds now clamp to `[-RevMax, 0.25]` instead of `[0, 0.25]`; default `RevMax=0.06m/s`.
- Fixed GrayLine wheel PWM feedforward to use each wheel's own target speed before applying the wheel-speed PID trim, so `TurnMax` produces immediate differential PWM instead of relying only on trim.
- TFT `PID -> GrayLine` now includes `RevMax`; `grayline pid` prints it too.

## 2026-07-18 - GrayLine nonlinear small-error gain

- Added nonlinear GrayLine outer P gain to reduce straight-line oscillation without weakening large-arc correction.
- When `abs(error) <= SmallBd`, the P term uses `Kp * SmallGn`; outside that band it uses full `Kp`.
- Defaults are `SmallGn=0.45` and `SmallBd=1.0`, so `line_pos=-1/0/+1` is softer while `|line_pos|>=2` keeps full turn authority.
- TFT `PID -> GrayLine` now edits `SmallGn` and `SmallBd`; `grayline pid` prints both.
- Raised TFT GrayLine tuning limits for field use: `Kp` max `0.50`, `TurnMax` max `0.30`, `BaseMps` max `0.25`.

## 2026-07-18 - GrayLine filtered position and ramp gain

- Replaced the hard small/full P gain switch with a gradual ramp: effective gain moves from `Kp*SmallGn` at zero error to full `Kp` at `abs(error)>=SmallBd`.
- Added first-order position filtering before the outer PID; `FiltA` is the new-sample weight, default `0.45`.
- Default `SmallBd` is now `2.0`, so gain rises smoothly across the `0..2` sensor-position range instead of jumping at `1..2`.
- VOFA `line_pos` now reports the filtered/control position used by the outer loop, so fractional line positions are expected.
- TFT `PID -> GrayLine` now includes `FiltA`; `grayline pid` prints it.

## 2026-07-18 - Temporary speed execution test UART

- Added temporary UART commands to isolate wheel-speed execution before further GrayLine tuning.
- Commands: `speedtest help`, `speedtest start <signed_pwm>`, `speedtest status`, `speedtest stop`.
- While running, the command task prints every 100ms: requested PWM, sample dt, left/right encoder counts, left/right m/s, and right/left speed ratio.
- Intended test: lift the car, run equal signed PWM such as `speedtest start 20`, capture 2-3 seconds of output, then `speedtest stop`.

## 2026-07-18 - GrayLine restored to 22:32 PID baseline

- Restored GrayLine to pure position outer PID plus dual wheel-speed inner PID.
- Removed the experimental `SmallGn`, `SmallBd`, and `FiltA` tuning fields from GrayLine control, UART PID print, and TFT PID menu.
- Defaults are `BaseMps=0.14`, `Kp=0.024`, `Ki=0`, `Kd=0.001`, `TurnMax=0.20`, `RevMax=0.06`, and `Slew=4.0`.
- Kept the manual `speedtest start/status/stop` commands for ground speed checks.

## 2026-07-19 - GrayLine PID loop explanation

- Current GrayLine control has one gray-position outer PID and two parallel wheel-speed inner PID calculations.
- The outer PID is edited through `PID -> GrayLine`; it converts `line_pos` error into `turn_mps`.
- The left/right inner speed PID uses the shared `WheelSpd` parameters from Motion, not a separate GrayLine-only parameter group.
- In practice, most field tuning so far changed the outer loop, `BaseMps`, `TurnMax`, `RevMax`, and `Slew`; the inner speed loop stayed at its default/shared `WheelSpd` settings.
- The inner loop output is limited by `PwmTrim`, so it is currently a correction on top of speed-to-PWM feedforward rather than the main source of steering authority.

## 2026-07-21 - Path reverse differential diagnostics

- Kept the shared PathTrack parameters and existing path replay output flow unchanged.
- Added reverse and signed differential speedtest modes plus expanded path direction/trim/PWM diagnostics.
- Keil build passed with `0 Error(s), 0 Warning(s)`; on-target comparison logs are pending.

## 2026-07-21 - INS command stack hardfault guard

- `speedtest reverse 20 12` measured `-166/-135 mm/s`, confirming existing reverse differential output.
- Increased `ins_cmd` stack from 160 to 256 words and added `stack_min_free` to `speedtest status` after a formatted-log HardFault.

## 2026-07-21 - Non-blocking UART logger

- Removed boot-faulting dynamic `log_tx` queue/task after PendSV saw `pxCurrentTCB=0x07070707`.
- UART0 TX interrupt now sends two static 128-byte complete records without character-level interleaving; a pending replacement increments `log_overwrite`.
- RX is 128 bytes for the 80-byte command line; FreeRTOS stack/malloc fault hooks directly report then reset. Path and motor control are unchanged.

## 2026-07-21 - UART asynchronous logging rollback

- Restored direct blocking UART and removed the async TX implementations after repeatable boot faults.
- RX is again 256 bytes and FreeRTOS stack/malloc hooks are disabled; speedtest stack margin remains.

## 2026-07-21 - GrayLine yaw-rate damping loop

- GrayLine now runs a 100 Hz yaw-rate damping loop using calibrated ICM gyro-Z data. It keeps the existing position PID feed-forward and adds a bounded yaw-rate correction.
- Default rate settings: `RateKp=0.0008`, `RateKi=0`, `RateKd=0`, `RateMax=180 dps`, `RateTrim=0.060 m/s`.
- Added five TFT parameters under `PID -> GrayLine`; no UART tuning command was added.
- JustFloat is now 17 channels at 50 Hz: original channels 0-9 are retained; channels 10-16 are position feed-forward, direct target rate, actual rate, rate error, rate trim, and active flag. Control remains 100 Hz because UART transmission is blocking.
- IMU read failure marks the rate inactive and safely continues the established position-only line-following behavior.
- Keil build passed with `0 Error(s), 0 Warning(s)`.

## 2026-07-21 - GrayLine TFT small-gain precision

- TFT float rendering now shows four decimal places for menu items with a step below `0.001`. `RateKp`, `RateKi`, and `RateKd` use a `0.0001` step, so `0.0012` is now visible and can be verified on screen.

## 2026-07-21 - GrayLine direct-turn correction

- Removed `RateSlew`: it incorrectly replaced the gray-position steering feed-forward with a limited yaw-rate command and made sharp curves understeer.
- GrayLine now sends the position-loop turn command directly to the wheels; the yaw-rate PID only adds a bounded damping trim. Existing PWM slew remains the physical output smoothing mechanism.

## 2026-07-21 - GrayLine feedback sign and encoder-spike correction

- Initial JustFloat correlation was not sufficient to override physical steering direction. GrayLine retains IMU Z-axis sign `+1`; physical on-target behavior is the source of truth.
- GrayLine wheel-speed PID now rejects a frame if either encoder-derived speed exceeds `0.80 m/s`, while advancing its encoder reference so the same corrupt counts are not reused.

## 2026-07-24 - Runtime log management

- Replaced legacy text log macros with runtime level/module filtering and UART `log` commands.
- JustFloat defaults off and is an explicit exclusive `motion` or `grayline` source.
- Keil rebuild currently stops in the existing SysConfig pre-build command because `C:\.metadata\product.json` is missing.

## 2026-07-25 - UART DMA text logger

- [x] Updated `empty.syscfg` and regenerated UART0 TX-DMA configuration: full DMA CH2, byte transfers, TX trigger, DMA_DONE_TX/EOT_DONE interrupt sources, and TX FIFO enabled.
- [x] Replaced blocking text logger with one 160-byte static whole-record DMA slot; busy text logs are dropped and counted by `LOG_GetDroppedCount()`.
- [x] Kept JustFloat exclusive and protected it from concurrent text-DMA or another telemetry frame.
- [x] Reduced `PATH_MAX_POINTS` from 500 to 292, giving over 2 KiB net RAM back after logger storage.
- [x] Enabled FreeRTOS stack-overflow level 2 and added a direct-UART overflow report/reset hook.
- [x] Keil clean rebuild completed: 0 errors, 0 warnings.
- [ ] On target: verify no boot HardFault, record integrity under concurrent task logs, dropped count under heavy logging, and stack-watermark margins.

## 2026-07-25 - Dual-path UART logger

- [x] Added serialized reliable DMA transport for initialization, ERROR, CLI replies, and stack reports; normal logs remain non-blocking DMA.
- [x] Enabled UART0 NVIC during hardware/log initialization so reliable init records can receive EOT before `ins_cmd` begins.
- [x] ERROR now stops selected JustFloat telemetry before reporting; telemetry must be selected again afterward.
- [x] Keil clean rebuild: 0 errors, 0 warnings; ZI=30056 bytes (+168 bytes).
- [ ] On target: cold boot full log, complete 100-second stack report, command help/status output, telemetry/error priority, and `min_ever >= 2048` bytes.

## 2026-07-25 - Compact boot log and Gray stack

- [x] Removed successful task-created/task-ready records; retained one worker summary and all create failures.
- [x] Reduced ICM/LIS boot output to final identity records; scans/probes DEBUG, failures ERROR.
- [x] Reduced Flash output to JEDEC/capacity and `Flash selftest PASS`; added explicit readback ERROR.
- [x] Reduced IMU calibration to keep-still, gyro bias, one MAG2D result, and WARN only when invalid.
- [x] Set Gray stack and monitor default to 154 words (+104 bytes).
- [x] Keil clean rebuild: 0 errors, 0 warnings (Code=101576, RO=16628, RW=440, ZI=30056).
- [ ] On target: cold boot and 100-second stack report; require Gray total=154 words and `min_ever >= 2048` bytes.
