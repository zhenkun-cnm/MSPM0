# 研究发现

## 2026-06-30: ICM-20608 驱动开发

### 手册分析
- ICM-20608 产品规格书（35 页）不包含完整寄存器映射表
- 手册明确引用 "ICM-20608-G Register Map" 作为独立文档
- 寄存器地址采用标准 MPU-6xxx 兼容映射（ICM-20608 与 MPU-6500/ICM-20602 寄存器兼容）

### SPI 协议要点（手册 §6.5 p.28）
- 数据格式: R/W(bit7) + 7bit 寄存器地址
- MSB 先，上升沿锁存（Mode 0: CPOL=0, CPHA=0）
- 支持 Mode 0 和 Mode 3 (p.13)
- 最大 8MHz (p.15)
- 支持单字节和突发读写

### 引脚复用确认（ti_msp_dl_config.h）
- SPI1: PA17(SCLK), PA18(PICO), PA16(POCI)
- Flash CS: PA15 (GPIO 控制)
- IMU CS: PA14 (IOMUX_PINCM36, 空闲)

### ICM-20608 与 ICM-20948 关键差异
1. 无 Bank 系统 — 寄存器直访
2. 6 轴 vs 9 轴（无磁力计）
3. 数据寄存器地址不同: ACCEL 0x3B vs 0x2D, GYRO 0x43 vs 0x33
4. WHO_AM_I: 0xAE vs 0xEA
5. I2C_IF_DIS 位相同（都是 USER_CTRL bit4）

### 初始化流程（按手册顺序）
1. 上电等待 ≥100ms（寄存器读写启动时间 p.11）
2. 软复位 PWR_MGMT_1 bit7（p.24）
3. CLKSEL=001 自动时钟（§4.10 p.22）
4. I2C_IF_DIS=1 禁用 I2C（§6.1 p.25）
5. 配置量程（Table 1 p.7, Table 2 p.8）

### 文件修改清单
| 文件 | 操作 | 说明 |
|------|------|------|
| port/inc/port_imu.h | 重写 | ICM-20948 → ICM-20608 |
| port/src/port_imu.c | 重写 | 软件SPI → 硬件SPI1 |
| task_plan.md | 更新 | 新计划 |
| ICM20608.datasheet.md | 新建 | 手册缓存（PDF 同目录） |

---

## 2026-07-15 程序恢复发现

- 本次丢失主要表现为 Keil 工程编译项回退：`app_ins.c`、`app_ins_cmd.c`、`app_motion.c`、`port_uart_rx.c` 没有进入 `.uvprojx` 编译列表。
- `app_init.c` 也回退到旧状态，缺少 IMU/INS/motion 队列创建和任务启动，已恢复。
- `app_stack_monitor.c` 使用 `xTaskGetHandle()`，当前 FreeRTOS 配置不声明该 API，会导致编译错误；已暂时撤出构建和启动。
- 恢复后 clean rebuild 通过，说明 INS、串口命令、motion、TFT PID 菜单和 UART RX port 已重新进入固件。
## 2026-07-17 - NAV v1 findings

- After straight PID and turn PID, the most useful next layer is point navigation.
- `app_nav` sequences existing `motion` primitives instead of adding arc control yet.
- `nav square` is the first whole-path regression test for INS plus motion.
- Current v1 tolerances: `0.05m` position, `3deg` yaw, `5.0m` max leg.
## 2026-07-17 - NAV square root cause

- `nav square` could skip steps because completion was inferred from motion IDLE state.
- Single-wheel turning changes `X/Y`, so square is now an action sequence instead of fixed ideal waypoints.
- Motion rejection/non-start is now visible through `rejected_cmd_id` and `last_result`.
## 2026-07-17 - Test1 decisions

- Test1 is a fixed competition route module, separate from generic NAV.
- Coordinates are centimeters converted to meters.
- Third target is `X=4cm, Y=-96cm`.
- Every waypoint is followed by `motion turn -90`.

## 2026-07-17 - Turn-after-drive diagnosis rules

- The observed failure "drive straight then stop, no turn" must be diagnosed by command chain, not by PID tuning first.
- `[TEST1_STEP] send right turn` proves the Test1 state machine issued the turn command.
- `[MOTION_ACK] accept ... type=TURN` proves motion accepted the turn command.
- If turn is accepted but the car does not physically rotate, the next likely layer is TB6612 single-wheel turn control, motor wiring/direction, PWM output, or brake/coast behavior.
- If motion accepts and reports done without physical turn, inspect yaw completion logic and IMU yaw validity.

## 2026-07-17 - Arc/path v1 findings

- Arc v1 should tune wheel-speed PID first; Test1 already validated straight and single-wheel turn enough to avoid retuning those loops immediately.
- IMU yaw is only the arc completion condition in v1, not an outer PID loop.
- VOFA JustFloat for arc uses 6 channels so wheel-speed tracking and yaw target/actual are visible together.
- Irregular arcs are safer as RAM teach/replay points first, before continuous path tracking is added.

## 2026-07-17 - TFT PID menu findings

- Arc v1 does not need a separate new PID parameter group yet. It reuses the existing wheel-speed PID because the first arc controller is only left/right wheel-speed inner loops.
- A dedicated `Arc Status` page is useful because VOFA requires enabling raw JustFloat output, while TFT can continuously show target/actual wheel speeds during ordinary bench tuning.
- `Yaw Status` now labels `MOTION_RT_ARC` as `ARC`, so the motion state display no longer collapses arc mode into IDLE.

## 2026-07-17 - Arc control finding

- The wheel-speed feedback is too quantized/noisy for strong speed PID or D-term tuning.
- For the current chassis, arc testing should prioritize stable fixed left/right PWM ratio plus small feedback correction.
- `Kd` should stay at 0 unless encoder velocity is filtered more heavily or replaced by more reliable hardware capture.

## 2026-07-17 - Arc asymmetry finding

- Right arcs are repeatable enough to use as the current baseline.
- Left arcs have acceptable yaw completion but a smaller effective radius/translation than right arcs.
- The first compensation should reduce left-turn curvature without changing right-turn behavior.

## 2026-07-17 - Teach/replay priority

- Since the target use case is push-once/replay, standard arc tuning only needs to be good enough to support segment replay.
- Point-to-point replay creates a polygonal path and loses the pushed arc shape.
- Segment replay using recorded yaw deltas plus `motion arc` should preserve arcs better than turn+drive replay.

## 2026-07-17 - Continuous replay decision

- Segment replay still creates visible pauses because each segment waits for `motion` completion.
- Continuous tracking should be the preferred path replay v2 because the user prioritizes smooth replay of a pushed curve over exact per-segment completion.
- Initial continuous tracking should tune lookahead/base PWM/yaw Kp before returning to wheel-speed PID work.

## 2026-07-17 - Path tracking curvature observation

- Logs showed persistent negative heading error around `-10..-15deg`, so the controller knew the target was inside the current heading but was not steering tightly enough.
- Smaller lookahead and higher yaw gain are the correct first knobs before changing base speed.

## 2026-07-17 - Path capacity finding

- The 64-point record limit was a fixed RAM buffer limit, not an algorithmic limit.
- Increasing to 160 points costs about 1.9KB total path-point storage, which is acceptable on the current build.
- If future paths need much more than 160 points, the next step should be adjustable downsampling or flash-backed recording rather than blindly growing RAM.

## 2026-07-18 - Path replay early-finish finding

- Long routes can pass physically near their final point before the logical replay index reaches the route tail.
- Therefore continuous replay completion must be gated by both final-point distance and replay progress.
- The current guard allows final-distance completion only once `replay_index >= s_pathCount - 3`.

## 2026-07-20 - M2 TIMG7 dual-capture DMA direction finding

- M2 uses PA28/TIMG7_CCP0 and PA31/TIMG7_CCP1 with two DMA timestamp buffers, then decodes in the 10 ms motor encoder task. This avoids edge interrupts and avoids DMA reads from GPIO input registers.
- A bug caused right-wheel forward and reverse rotation to both count positive: `DL_TimerG_setCaptureCompareInput()` was called with `DL_TIMER_INPUT_CHAN_0/1` as the third argument. That argument is not a channel index; it is a `DL_TIMER_CC_IN_SEL_*` input selector.
- Passing `DL_TIMER_INPUT_CHAN_1` made CC1 select the paired input, so CC0 and CC1 effectively captured the same edge stream. UART diagnostics showed `Ao == Bo`, `Ac == Bc`, and `Eq == Ac/Bc`, proving both DMA buffers received equal timestamps.
- The fix is to configure both capture units with `DL_TIMER_CC_IN_SEL_CCPX`, matching SysConfig generated code, so CC0 uses its own CCP0 input and CC1 uses its own CCP1 input.
- If this failure returns, first check the diagnostic fields: repeated `Eq` equal to the A/B event counts means the problem is event/input selection, not direction sign. Only adjust `ENC2_DIR_SIGN` after A/B timestamps are distinct and forward/reverse are consistently opposite.

## 2026-07-20 - Wheel encoder calibration update

- Current user-measured calibration is 1054 counts per output revolution for both left and right wheels.
- The theoretical value remains 1040 counts per output revolution, but runtime speed and distance conversion must use the measured 1054 value.
- The old right-wheel calibration near 985 counts per output revolution is obsolete and must not be used for current INS, Motion, GrayLine, or speedtest calculations.
- INS odometry remains signed: backward wheel counts are allowed to reduce `left_m`, `right_m`, and `X/Y` through a negative `ds`.

## 2026-07-20 - Path replay reverse differential finding

- The current backward replay problem is not an INS integration problem: INS already supports negative `ds`, negative `v_mps`, and signed `X/Y` updates.
- The path replay controller is the limiting layer. It can detect reverse segments and print `dir=B`, but reverse tracking still relies on global `TB6612_DIR_REVERSE` plus positive left/right PWM.
- The motor command layer already has independent signed wheel commands: `MOTOR_CMD_LEFT_SIGNED_SPEED` and `MOTOR_CMD_RIGHT_SIGNED_SPEED`. These are the correct mechanism for backward differential turning.
- To protect the currently working forward replay behavior, the first implementation should keep forward segments on the existing output path and use signed PWM only for reverse segments.
- Parameters need to be made tunable before field tuning: PathTrack PID belongs on TFT, while lookahead/base PWM/trim/PWM limits/done distance should be adjusted by UART commands.

## 2026-07-21 - Reverse differential diagnostic decision

- Existing reverse replay already commands unequal PWM with a global reverse direction, so negative signed PWM alone is not assumed to improve steering.
- PathTrack parameters remain shared between forward and reverse; reverse and signed speedtests will provide encoder-backed evidence before any replay control-law change.

## 2026-07-21 - INS command stack finding

- Global reverse `20/12` produced measured `-166/-135 mm/s`; the differential motor output works.
- After a PC=0 HardFault during the next long speedtest log, the `ins_cmd` stack was increased from 160 to 256 words and its high-water margin was exposed in `speedtest status`.

## 2026-07-21 - UART logger RAM and serialization finding

- The requested static 12 x 128-byte TX ring could not link: remaining RAM was short by 1504 bytes.
- The dynamic queue/task fallback failed at boot: PendSV read `pxCurrentTCB=0x07070707`, a FreeRTOS initial-stack register fill value.
- Two static 128-byte TX records now use UART0 TX interrupt; only the unsent pending record may be replaced, and `LOG_GetOverwriteCount()` reports it.

## 2026-07-21 - UART asynchronous logging rollback

- User selected stability after repeated boot faults. TX-interrupt and queue/task log variants are removed.
- Direct blocking UART, 256-byte RX, and disabled stack/malloc hooks are restored.

## 2026-07-21 - GrayLine yaw-rate damping decision

- A 10 ms IMU update is retained because GrayLine itself runs at 10 ms; increasing only sensor reads would not improve actuator-loop bandwidth.
- The yaw-rate loop is used as damping around the gray-position steering feed-forward. At straight-line target rate zero, it opposes unintended yaw; in a corner the gray-position command remains direct so sharp-turn authority is preserved.
- Direct calibrated gyro-Z is preferable to INS `w_dps`, which is derived from yaw differences and may include magnetometer/estimator effects.
- The 17-float JustFloat frame is 72 bytes and direct UART transmission is blocking. Telemetry is therefore limited to 50 Hz; control remains 100 Hz. Raising telemetry to 100 Hz would occupy about 6.25 ms of every 10 ms control period.

## 2026-07-21 - GrayLine yaw-sign validation

- With direct position-loop steering, JustFloat showed a negative correlation between GrayLine turn command and IMU Z rate, but physical steering verification showed that inverting gyro-Z was wrong. GrayLine retains the original IMU Z sign; INS yaw convention is unchanged.
- One captured encoder frame reported an impossible `-6396 m/s` left-wheel speed. GrayLine now rejects encoder-derived speeds above `0.80 m/s` before its wheel-speed PID consumes them.

## 2026-07-24 - Logging findings

- Legacy `LOG_RAW` and direct formatter bypasses were replaced by level/module macros.
- Blocking UART transport remains intentionally unchanged because prior asynchronous variants HardFaulted.

## 2026-07-25 - UART DMA logging and PendSV fault finding

- The supplied fault frame had `LR=0x4231`, which maps into `PendSV_Handler`; `PC=0` means a saved task context restored a null program counter. It is not evidence that a UART byte-transfer interrupt itself jumped to zero.
- The old logger allocated 128-byte message plus 160-byte output arrays on every caller's task stack before calling formatted output. Multiple boot tasks have only 96--192 words of stack, so this made stack corruption plausible. Character-level UART interleaving proved concurrent access but did not alone explain PC=0.
- Text logging is now best-effort DMA: one static 160-byte complete record, one whole DMA transfer, busy producer drops the whole record. There is no log queue, log task, TX FIFO ISR, dynamic allocation, or partial-record software transmit loop.
- UART DMA completion only releases the slot on EOT, after the final hardware byte leaves the UART. RX remains serviced by the same UART0 IRQ handler.
- Map verification: path capacity reduced 500 -> 292 points, and `s_pathPoints` is 3504 bytes. With 12-byte points this frees 2496 bytes; the logger transport adds 166 bytes beyond existing log state, leaving 2330 bytes net free RAM.
- `configCHECK_FOR_STACK_OVERFLOW=2` and `vApplicationStackOverflowHook` are enabled. The hook intentionally uses direct UART because the normal logger may be the suspected failure path.

## 2026-07-25 - Reliable log delivery finding

- Startup loss was caused by `start_task` keeping interrupts disabled while issuing many ordinary DMA logs: EOT could not clear the sole busy flag. A 100-second stack report similarly issued about 19 records faster than 115200 baud can transmit.
- A dedicated TX task is unnecessary for the selected policy. Reliable callers wait on UART EOT through a mutex and task notification; the ISR only handles completion and never feeds partial records.
- Reliable paths are intentionally restricted to boot, ERROR, command replies, and the low-priority stack monitor. Arbitrarily high-rate normal logs remain drop-on-busy so motor/navigation timing is not coupled to UART throughput.

## 2026-07-25 - Boot log reduction rationale

- Per-task creation and `task ready` records repeat the successful creation path. One `Worker tasks created` record keeps the useful boot milestone without a burst of reliable UART waits.
- ICM-20608 and LIS3MDL final records include selected I2C address and WHO_AM_I. SDA/SCL state, scans, probes, and ACK listings are DEBUG-only; transfer and initialization failures remain ERROR.
- Flash now emits one JEDEC/capacity summary and `Flash selftest PASS`; erase/program/readback phases are silent unless an operation fails.
- Gray receives 154 words (616 bytes), exactly 20% above 128 words (512 bytes). On-target stack report must still show `min_ever >= 2048` bytes.

## 2026-07-26 - Path/gray fusion decisions

- The "inertial direction" for this feature is the target direction and heading correction calculated by `APP/src/app_path.c`, not raw gyro-Z direction.
- In forward path segments, matching non-zero grayscale and path PWM corrections blend as 80% gray / 20% path. Opposite direction, one-side-straight, stale/no-line, and reverse segments use 100% path correction.
- The shared motor queue is protected at the APP level by a mode owner check; GrayLine no longer writes motor PWM while it is a path-fusion assistant.

## 2026-07-26 - Fusion startup fallback

- The previous `PATH_STATE_FUSION_ARMING` gate held the vehicle for up to 500 ms waiting for a fresh black-line frame, then raised `PATH_ERROR_GRAY_UNAVAILABLE`.
- Fusion now starts in `PATH_STATE_REPLAY_TRACK` with diagnostic mode `STALE`; the normal runtime guard is the single source of truth for switching between path-only and blend.

## 2026-07-27 - UART2/ICM isolation checkpoint

- ICM status `0x00010026` means address-phase NACK before `WHO_AM_I`; it is not an ID mismatch.
- Fusion baseline and the prior failing branch have identical I2C0 PA0/PA1, 400 kHz timing, ICM/LIS3MDL drivers, and IMU priority.
- This image only configures UART2 PB17/PB18 and leaves the UART2 NVIC disabled, isolating UART2 hardware configuration from protocol, task, and heap changes.

## 2026-07-27 - Size-optimization boundary

- Full communication runtime exceeded Flash by 0xF88 with baseline optimization.
- `-Oz` is now limited to App and Algorithm groups, producing a 110536-byte Flash image; target/global optimization remains unset.
- `app_imu.c` and `app_init.c` use `#pragma clang optimize off`; ICM Port files are not in optimized groups.

## 2026-07-27 - Vehicle 2 protocol identity

- Vehicle 2 is published as GitHub branch `vehicle2`; Vehicle 1 is `big-car` in the same target repository.
- The shared `VEHICLE_SYNC.md` fixes the wire contract: UART2 115200, node IDs `0x01`/`0x02`, PING `0x01`, ACK `0xF0`, 20-byte maximum payload, and XOR-protected framing.
- `CAR_COMM_NODE_ID=2U` is an App-group Keil compiler definition, so the shared source header keeps its default node value without causing an ID collision.
- Vehicle 1's 8-entry queues and Vehicle 2's 2-entry queues are local buffering choices, not a protocol mismatch.
