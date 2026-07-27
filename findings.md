# 研究发现 - MSPM0G3507 FreeRTOS 嵌入式系统

## 架构发现

### 1. 启动流程
```
main()
  ├── PORT_SYSTEM_Init()    → SYSCFG_DL_init() (HAL 层初始化)
  ├── app_init()             → 创建 start_task
  └── app_start()            → vTaskStartScheduler() (启动 FreeRTOS)

start_task:
  ├── 创建 led_task (prio=2, 128w)
  ├── 创建 encoder_task (prio=2, 256w)
  ├── 创建 flash_init_task (prio=1, 256w)
  └── 创建 imu_task (prio=2, 384w)
  └── LOG_INFO 系统启动信息
  └── vTaskDelete(NULL) 删除自身
```

### 2. Device 层 OOP 设计模式
- Device 层使用函数指针结构体模拟 OOP `vtable`
- 每个设备一个全局 `GetXxx()` 工厂函数返回单例
- 接口编译期绑定，运行时零虚函数开销

### 3. ICM-20948 软件 SPI 实现
- 使用 GPIO bit-bang 模拟 SPI
- CPOL=0, CPHA=0 (空闲 SCK=0, 上升沿锁存)
- 约 150kHz SCLK @ 3us 半周期（受 LSF0108 电平转换器 RC 边沿限制）
- 支持 Bank 切换 (寄存器分 Bank 0/2)
- WHO_AM_I 读回验证 (预期 0xEA, 5次重试)
- 上电后禁用 I2C 从机接口，强制 SPI 模式

### 4. W25Q128 Flash 实现
- 使用 SPI1 硬件外设 (非 bit-bang)
- 完整实现：JEDEC ID 读取、扇区擦除 (4KB)、页编程 (≤256B)、读取
- 写使能 + BUSY 等待保护
- app_flash.c 包含完整的 **擦除→写入→读出验证** 流程

### 5. EC11 编码器实现 (464 行代码)
- **旋转**: Gray 码查表解码 + 软件消抖（3次稳定确认）+ 软件2分频
- **按键**: 5态 FSM（IDLE/DEBOUNCE_DOWN/PRESSED/DEBOUNCE_UP/WAIT_DOUBLE）
- 支持：短按、长按 (≥1000ms)、双击 (≤250ms间隔)
- 旋转事件单槽模式，按键事件环形队列 (16深)
- 引脚：PA24 (A相), PA25 (B相), PA26 (按键)

### 6. FreeRTOS 配置
- 使用 heap_4 内存管理
- 所有任务栈大小: 128~384 字 (word, 4 bytes each)
- 总 RAM 估算: ~1.5KB 任务栈 + 系统开销

---

## 完整文件清单

### APP 层 (c:\ti\mspm0_project\APP\)
| 文件 | 路径 | 功能 |
|------|------|------|
| app_init.h / .c | APP/inc/, APP/src/ | start_task 创建与调度 |
| app_led.h / .c | APP/inc/, APP/src/ | LED 闪烁任务 |
| app_encoder.h / .c | APP/inc/, APP/src/ | 编码器轮询事件消费 |
| app_flash.h / .c | APP/inc/, APP/src/ | Flash JEDEC + 擦写读验证 |
| app_imu.h / .c | APP/inc/, APP/src/ | IMU 6轴数据采集 (1Hz) |

### Device 层 (c:\ti\mspm0_project\Device\inc\)
| 文件 | 方法数 | 接口 |
|------|--------|------|
| dev_led.h | 5 | init, on, off, toggle, getState |
| dev_encoder.h | 4 | init, getEvent, getPosition, resetPosition |
| dev_flash.h | 5 | init, readJEDECID, sectorErase, pageProgram, read |
| dev_imu.h | 3 | init, whoAmI, readSensorData |

### Port 层 (c:\ti\mspm0_project\board\empty\port\)
| 文件 | 驱动方式 | 代码行数 | 实现设备 |
|------|---------|---------|---------|
| port_system.c | SYSCFG_DL_init | 19 | 系统时钟/GPIO/UART |
| port_led.c | DL_GPIO | (未读) | LED 控制 |
| port_encoder.c | DL_GPIO 轮询 | 464 | EC11 编码器 + 按键 |
| port_flash.c | SPI1 硬件 | 225 | W25Q128 NOR Flash |
| port_imu.c | GPIO bit-bang SPI | 351 | ICM-20948 9轴传感器 |
| port_log.c | UART printf | (未读) | 日志输出 |

### 构建输出
- `Objects/` 目录存在所有 `.o` 文件，**项目编译通过**

---

## 模块完整性验证结果

| 模块 | Device 层 | Port 实现 | App 任务 | 完整性 |
|------|-----------|-----------|---------|--------|
| **System** | - | ✅ | ✅ | ✅ 完成 |
| **LED** | ✅ dev_led.h | ✅ port_led.c | ✅ app_led.c | ✅ 完成 |
| **Encoder** | ✅ dev_encoder.h | ✅ port_encoder.c | ✅ app_encoder.c | ✅ 完成 |
| **Flash** | ✅ dev_flash.h | ✅ port_flash.c | ✅ app_flash.c | ✅ 完成 |
| **IMU** | ✅ dev_imu.h | ✅ port_imu.c | ✅ app_imu.c | ✅ 完成 |

---

## 硬件引脚总表

| 外设 | 引脚 | 功能 |
|------|------|------|
| LED | (待确认) | 板载 LED |
| EC11 | PA24 | 编码器 A 相 (CLK) |
| EC11 | PA25 | 编码器 B 相 (DT) |
| EC11 | PA26 | 编码器按键 (SW) |
| W25Q128 | SPI1 (CS 见 ti_msp_dl_config) | SPI Flash |
| ICM-20948 | PB9 | SCK (软件 SPI) |
| ICM-20948 | PB8 | MOSI |
| ICM-20948 | PB7 | MISO |
| ICM-20948 | PB6 | CS |

---

## 已知风险

| 风险 | 严重程度 | 描述 |
|------|---------|------|
| 软件 SPI 时序 | 中 | bit-bang SPI 可能在高温/低压下时序偏差 |
| LSF0108 电平转换 | 中 | RC 边沿限制 SPI 频率上限 ~400kHz |
| 堆栈溢出 | 低 | 需实测确认各任务栈使用量 |
| Flash 写磨损 | 低 | 当前仅有验证测试，无频繁写入 |

---

## 2026-07-15 程序恢复发现

- 本次丢失主要表现为 Keil 工程编译项回退：`app_ins.c`、`app_ins_cmd.c`、`app_motion.c`、`port_uart_rx.c` 没有进入 `.uvprojx` 编译列表。
- `app_init.c` 也回退到旧状态，缺少 IMU/INS/motion 队列创建和任务启动，已恢复。
- `app_stack_monitor.c` 使用 `xTaskGetHandle()`，当前 FreeRTOS 配置不声明该 API，会导致编译错误；已暂时撤出构建和启动。
- 恢复后 clean rebuild 通过，说明 INS、串口命令、motion、TFT PID 菜单和 UART RX port 已重新进入固件。
## 2026-07-17 - NAV v1 findings

- The correct next step after straight PID and turn PID is a navigation layer, not arcs yet. `app_nav` now sequences proven motion primitives using INS pose.
- First implementation keeps the control problem observable: heading alignment, straight distance, and final yaw are tested separately inside one `nav goto`.
- `nav square` is a higher-level regression test for accumulated INS and motion error. It reuses INS Flash logs for replay.
- Current limits are conservative: position tolerance `0.05m`, yaw tolerance `3deg`, maximum single leg `5.0m`.
## 2026-07-17 - Why `nav square` only drove one line

- The old NAV-to-motion completion check could treat `MOTION_RT_IDLE` as completion even if the next motion command never actually started.
- Single-wheel turning changes `X/Y`, so square should not be implemented as ideal fixed waypoints for this chassis.
- The fix is command-level handshake plus square-as-action-sequence: drive, turn, drive, turn.
## 2026-07-17 - Test1 route decisions

- Test1 is implemented as a separate competition route module, not as `nav square`.
- User-provided coordinates are centimeters; code stores meters.
- Third point is `X=4cm, Y=-96cm`.
- Right turns use relative `motion turn -90`.

## 2026-07-17 - Turn-after-drive diagnosis rules

- The observed failure "drive straight then stop, no turn" must be diagnosed by command chain, not by PID tuning first.
- `[TEST1_STEP] send right turn` proves the Test1 state machine issued the turn command.
- `[MOTION_ACK] accept ... type=TURN` proves motion accepted the turn command.
- If turn is accepted but the car does not physically rotate, the next likely layer is TB6612 single-wheel turn control, motor wiring/direction, PWM output, or brake/coast behavior.
- If motion accepts and reports done without physical turn, inspect yaw completion logic and IMU yaw validity.

## 2026-07-17 - Arc/path v1 decisions

- Standard arc testing belongs in `app_motion` first, not in `app_nav`, because it verifies the chassis can hold a differential wheel-speed ratio before higher-level path logic depends on it.
- Arc v1 intentionally has no yaw/path outer PID: the only PID loops are left/right wheel-speed inner loops. IMU yaw stops the command when the requested angle is reached.
- VOFA tuning should focus on 6 JustFloat channels: target/actual left speed, target/actual right speed, target/actual yaw.
- Irregular arcs are implemented as teach/replay discrete points in RAM first. This keeps the first replay implementation debuggable and avoids introducing continuous path tracking before standard arcs are observed.

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

- The existing reverse replay already commands unequal left/right PWM under `TB6612_DIR_REVERSE`; replacing equal magnitudes with negative signed PWM alone cannot create additional differential steering authority.
- The chassis uses the same PathTrack parameters in both directions. No reverse-only RAM configuration or `path tune` interface was added.
- The next evidence comes from encoder-backed comparison: global reverse unequal PWM versus equal-magnitude signed PWM. Path replay will remain unchanged until those logs identify a controller, motor-output, or encoder issue.

## 2026-07-21 - Reverse differential result and UART stack finding

- `speedtest reverse 20 12` measured `-166/-135 mm/s`; the existing global reverse plus unequal regular PWM already produces an actual reverse speed difference.
- A subsequent long speedtest log caused an M0+ HardFault with `PC=0`. The `ins_cmd` task had only 160 words despite retaining an 80-byte RX line and calling formatted logging with a 128-byte local buffer.
- The task stack is increased to 256 words and `speedtest status` reports its high-water free words. Do not infer that `LOG_INFO` itself is the fault: the failing call used `LOG_RAW`, and both macros share the same formatter.

## 2026-07-21 - UART logger RAM and serialization finding

- A 12-record x 128-byte static TX ring plus an active record could not link in the current image: the linker reported a 1504-byte RAM shortfall.
- The dynamic queue/task fallback then produced an immediate boot HardFault. `PC=0x3DE6` is the PendSV read of the selected TCB, and `pxCurrentTCB=0x07070707` is an initial task-stack register fill value; this is scheduler-state corruption, not a UART peripheral fault.
- The replacement is two static 128-byte records with UART0 TX interrupt service: one partially/actively sending record and one newest pending record. `LOG_GetOverwriteCount()` increments only when an older unsent pending record is replaced.

## 2026-07-21 - UART asynchronous logging rollback

- The user chose stability over non-blocking logs after repeated boot-time faults.
- All asynchronous logging variants were removed. UART output is restored to the original direct blocking implementation; stack/malloc hooks are restored to their prior disabled configuration.

## 2026-07-21 - GrayLine yaw-rate damping decision

- A 10 ms IMU update is retained because GrayLine itself runs at 10 ms; increasing only sensor reads would not improve actuator-loop bandwidth.
- The yaw-rate loop is used as damping around the gray-position steering feed-forward. At straight-line target rate zero, it opposes unintended yaw; in a corner the gray-position command remains direct so sharp-turn authority is preserved.
- Direct calibrated gyro-Z is preferable to INS `w_dps`, which is derived from yaw differences and may include magnetometer/estimator effects.
- The 17-float JustFloat frame is 72 bytes and direct UART transmission is blocking. Telemetry is therefore limited to 50 Hz; control remains 100 Hz. Raising telemetry to 100 Hz would occupy about 6.25 ms of every 10 ms control period.

## 2026-07-21 - GrayLine yaw-sign validation

- With direct position-loop steering, JustFloat showed a negative correlation between GrayLine turn command and IMU Z rate, but physical steering verification showed that inverting gyro-Z was wrong. GrayLine retains the original IMU Z sign; INS yaw convention is unchanged.
- One captured encoder frame reported an impossible `-6396 m/s` left-wheel speed. GrayLine now rejects encoder-derived speeds above `0.80 m/s` before its wheel-speed PID consumes them.

## 2026-07-24 - Logging findings

- The old implementation used 298 `LOG_RAW` calls and direct formatter bypasses, so compile-time levels did not meaningfully classify field diagnostics.
- Blocking UART transport is deliberately retained: previous queue/task based logger variants caused boot-time HardFaults.
- JustFloat now requires a named active producer, preventing Motion and GrayLine frames from interleaving with each other or text logs.

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

## 2026-07-27 - Vehicle link publication

- Vehicle 1 is published as GitHub branch `big-car`; Vehicle 2 is published as `vehicle2`.
- `VEHICLE_SYNC.md` is mirrored in both repository roots. It is the required registry for UART2 frame changes, node IDs, commands, task ownership, and joint test evidence.
- Vehicle 1 source and UART2 port transport were present but not in the Keil compile list; both are now registered and the APP startup creates the communication task.
- The non-IMU size-optimization policy remains active. Adding the link leaves Vehicle 1 at a verified 0-error/0-warning build; physical communication is not yet verified.
