# ICM-20608 采集程序实现计划

## 目标
将现有 ICM-20948 软件 SPI 驱动替换为 ICM-20608 硬件 SPI1 驱动（与 W25Q64 共享总线）

## 关键决策
| 项目 | 决策 |
|------|------|
| SPI 驱动方式 | 硬件 SPI1 (与 W25Q64 共享) |
| IMU CS 引脚 | PA14 (IOMUX_PINCM36) |
| 传感器类型 | 6 轴 (陀螺+加计)，无磁力计 |
| 旧 ICM-20948 代码 | 全部删除，完全重写 |
| SPI 速率 | 与 W25Q64 现有配置兼容 |
| 寄存器映射 | 标准 MPU-6xxx 兼容（ICM-20608 手册不含完整表） |

## 阶段

### 阶段 1: 解析 ICM-20608 数据手册 ✅
- ✅ 读取 PDF 全 35 页
- ✅ 提取 SPI 接口协议 (p.28)、时序 (p.15)、寄存器地址、初始化流程
- ✅ 写入 `ICM20608.datasheet.md` 缓存

### 阶段 2: 重写 port_imu.h ✅
- ✅ 删除 ICM-20948 专属声明（Bank 切换、软件 SPI 宏）
- ✅ 新增硬件 SPI 函数声明
- ✅ CS 引脚 PA14 定义
- ✅ 保留 DevIMU 接口兼容

### 阶段 3: 重写 port_imu.c ✅
- ✅ CS GPIO 控制 (PA14)
- ✅ SPI 单字节收发（复用 W25Q64_INST = SPI1）
- ✅ 寄存器读写（手册 §6.5: R/W + 7bit addr）
- ✅ 突发读（支持连续读取多字节）
- ✅ 初始化序列:
  - 上电等待 100ms (p.11)
  - 软复位 PWR_MGMT_1 bit7 (p.24)
  - 退出睡眠 CLKSEL=1 (p.22)
  - 禁用 I2C (I2C_IF_DIS) (p.25)
  - 配置量程 ±2g / ±250°/s (p.7-8)
- ✅ WHO_AM_I 校验 (0xAE)
- ✅ 传感器数据读取（14 字节突发读，6 轴解析）

### 阶段 4: 验证编译 🔄
- 🔄 确认编译通过（需用户执行 Keil build）

### 阶段 5: 更新 Keil 工程
- 确认 port_imu.c 在编译列表中
- 确认 `#include "dev_imu.h"` 路径正确

## 遇到的错误
| 错误 | 尝试次数 | 解决方案 |
|------|---------|---------|
| 无 | 0 | - |
## 2026-07-17 - Current task: NAV v1

Status: implemented and build-verified.

Completed:
- Added `app_nav` files and task.
- Added `g_navCmdQueue`.
- Added UART commands: `nav help`, `nav status`, `nav stop`, `nav goto`, `nav square`.
- Registered `app_nav.c` in Keil `.uvprojx`.

Next tests:
- `ins reset`
- `nav goto 0.5 0.0 0`
- `ins reset`
- `nav goto 0.0 -0.5 -90`
- `ins reset`
- `nav square 0.6`
## 2026-07-17 - NAV square fix

Status: implemented and build-verified.

Test next:
- `ins reset`
- `nav square 0.6`
- Confirm logs show four drive steps and four turn steps.
- Use `nav status` / `motion status` if any step is rejected or does not start.
## 2026-07-17 - Test1 fixed route

Status: implemented and build-verified.

Test next:
- `ins reset`
- `ins log on`
- `test1 start`
- `test1 status` if needed.

## 2026-07-17 - Test1/nav turn-after-drive diagnostic fix

Status: implemented and build-verified.

Completed:
- Added `[MOTION_ACK] accept/reject/done` hard echoes in motion.
- Added `[TEST1_STEP]` transition logs around drive completion and right-turn command sending.
- Added `[NAV_STEP]` transition logs around square drive completion and turn command sending.

Next validation:
- `ins reset`
- `test1 start`
- Confirm first straight leg is followed by `[TEST1_STEP] send right turn` and `[MOTION_ACK] accept ... type=TURN`.
- If physical turning still does not happen after accept, move diagnosis to TB6612/motor command layer.

## 2026-07-17 - Arc/path v1

Status: implemented and build-verified.

Completed:
- Added standard arc command: `motion arc <radius_m> <angle_deg>`.
- Added 6-channel VOFA JustFloat output for arc PID tuning.
- Added `app_path` RAM teach/replay module.
- Registered `app_path.c` in Keil project files.

Next validation:
- `ins reset`
- `ins log on`
- `motion arc 0.50 90`
- `ins log off`
- `ins log print`
- Repeat with `motion arc 0.50 -90`.
- For irregular paths: `path record start`, push path, `path record stop`, `path print`, `ins reset`, `path replay`.

## 2026-07-17 - TFT PID menu for arc tuning

Status: implemented and build-verified.

Completed:
- Renamed top-level speed PID menu entry to `WheelSpd`.
- Added `PID -> Arc -> WheelSpd`.
- Added `PID -> Arc -> Arc Status`.
- Exposed arc target/actual wheel-speed runtime fields for the TFT page.

Next validation:
- Flash firmware.
- Open TFT menu: `PID -> Arc -> WheelSpd`, tune Kp/Ki/Kd/PwmTrim.
- Open TFT menu: `PID -> Arc -> Arc Status`, then run `motion arc 0.50 90`.
- Confirm left/right target speeds are nonzero and actual speeds change while the arc is running.

## 2026-07-17 - Arc weak closed-loop mode

Status: implemented and build-verified.

Completed:
- Filtered arc wheel-speed feedback before PID use.
- Capped arc PID trim to +/-4 PWM points.
- Added arc PWM slew limit of 2 PWM points per 10ms update.
- Changed default wheel-speed PID to `80/0/0/10`.

Next validation:
- Flash firmware.
- Run `motion status` and confirm `WheelSpd Kp=80.00 Ki=0.00 Kd=0.00 PwmTrim=10.0` after reset.
- Run `ins reset`, then `motion arc 0.50 -90`.
- Judge by physical smoothness and yaw completion first; treat wheel-speed `actual` only as a rough indicator.

## 2026-07-17 - Left arc compensation

Status: implemented and build-verified.

Completed:
- Added `MOTION_ARC_LEFT_INNER_SCALE = 1.10`.
- Added `MOTION_ARC_LEFT_OUTER_SCALE = 0.95`.
- Applied compensation only when `angle_deg > 0`.

Next validation:
- Run three trials of `ins reset`, `motion arc 0.50 90`.
- Compare with previous left arc baseline: `X=0.34..0.43, Y=0.39..0.42, yaw=95..96`.
- Right arc does not need retesting unless left compensation changes shared behavior unexpectedly.

## 2026-07-17 - Path replay as arc segments

Status: implemented and build-verified.

Completed:
- Added path motion send support for `value2`.
- Changed `path replay` to infer arc radius from consecutive recorded points and yaw delta.
- Falls back to `motion fwd` for near-straight segments and `motion turn` for tiny in-place yaw-only segments.

Next validation:
- `ins reset`
- `path clear`
- `path record start`
- Push one smooth arc or S curve.
- `path record stop`
- `path print`
- Put car back at the recorded start pose.
- `ins reset`
- `path replay`

## 2026-07-17 - Continuous path replay v2

Status: implemented and build-verified.

Completed:
- Added `PATH_STATE_REPLAY_TRACK`.
- `path replay` now directly controls TB6612 left/right PWM continuously.
- Added lookahead heading tracking and PWM slew limiting.
- Added tracking fields to `path status` and `[PATH_TRACK]` logs.

Next validation:
- `ins reset`
- `path clear`
- `path record start`
- Push one smooth arc or S curve.
- `path record stop`
- `path print`
- Put car back at the recorded start pose and direction.
- `ins reset`
- `path replay`
- Watch for no obvious stop-and-go behavior and record final translation error.

## 2026-07-17 - Path tracking curvature tighten

Status: implemented and build-verified.

Completed:
- `PATH_TRACK_LOOKAHEAD_M = 0.10`
- `PATH_TRACK_YAW_KP = 0.35`
- `PATH_TRACK_COMPLETE_DIST_M = 0.06`

Next validation:
- Replay the same recorded arc/S curve again.
- Check whether final yaw is closer to recorded final yaw and whether the replay arc radius is less oversized.

## 2026-07-17 - Path record capacity increase

Status: implemented and build-verified.

Completed:
- `PATH_MAX_POINTS = 160`
- Path count and replay indexes widened to `uint16_t`.
- No lower-priority task memory reduction was needed; build RAM summary remained acceptable.

Next validation:
- Run `path clear`, `path record start`, and push a longer mixed line/arc path.
- Confirm `path record stop` reports more than 64 points when the pushed path is long enough.
- Keep the first long test near 150 points before considering a larger buffer.

## 2026-07-18 - Path replay early-finish guard

Status: implemented and build-verified.

Completed:
- Added `PATH_TRACK_DONE_INDEX_BACKOFF = 3`.
- Continuous replay no longer finishes from final-point distance alone unless the replay index is within the last 3 points.
- Added `final=` to `path status` and `near final ignored` diagnostic logs.

Next validation:
- Replay the 166-point saved path and confirm it does not stop near `idx=32/166`.
- Completion should occur only near the route tail.

## 2026-07-18 - Path 500 points and stack trim

Status: implemented and build-verified.

Completed:
- `PATH_MAX_POINTS = 500`.
- Main startup stack reduced to `0x0800`; FreeRTOS heap remains 18KB.
- Path Flash save/load now uses two 4KB sectors.
- Low-usage task stacks were trimmed; stack monitor now prints heap free/min-ever.

Next validation:
- Confirm startup has no HardFault.
- Check stack monitor free words after normal path record/replay/save/load.
- Record and save a path longer than 330 points.

## 2026-07-21 - Path reverse differential diagnostics

Status: implemented and build-verified; physical verification pending.

- Shared PathTrack parameters and path replay output remain unchanged.
- Added `speedtest reverse <left_pwm> <right_pwm>` and `speedtest signed <left_pwm> <right_pwm>` for encoder-backed comparison.
- Added expanded `[PATH_TRACK]` and `[PATH_DIR]` diagnostics.
- Next: capture the four reverse/signed tests, then replay a mixed forward/reverse path.

## 2026-07-21 - INS command stack hardfault guard

- Increased `ins_cmd` stack 160 -> 256 words.
- `speedtest status` now prints `stack_min_free`.
- Validate a 3-second `speedtest reverse 20 12` run without HardFault before further reverse tests.

## 2026-07-21 - Non-blocking UART logging

- Replaced the boot-faulting dynamic `log_tx` queue/task with two static 128-byte records driven by UART0 TX interrupt.
- The active ISR record is never changed; a later record replaces only the pending record and increments `log_overwrite`.
- RX buffer is 128 bytes for the 80-byte command line. Stack-overflow and malloc-failed hooks report directly through UART then reset. No PathTrack or motor control values changed.
- Pending target check: five-second reverse tests must show complete records, no HardFault, positive stack margin, and `log_overwrite=0`.

## 2026-07-21 - UART asynchronous logging rollback

- User requested rollback after repeatable boot-time faults.
- Restored direct blocking UART, 256-byte RX, and default FreeRTOS hook configuration; removed TX-interrupt and queue/task logger code.
- Kept speedtest diagnostics and `ins_cmd` stack margin reporting.

## 2026-07-21 - GrayLine yaw-rate damping loop

Status: implemented and Keil build-verified; on-target tuning pending.

- Added a 10 ms IMU Z-axis yaw-rate feedback loop to GrayLine. The existing gray-position PID remains the steering feed-forward, while yaw-rate PID supplies a bounded damping trim.
- Added filtered gyro feedback for smoother straight-line behavior while retaining direct gray-position turn authority in corners.
- Added TFT `PID -> GrayLine` entries: `RateKp`, `RateKi`, `RateKd`, `RateMax`, and `RateTrim`.

## 2026-07-21 - GrayLine direct-turn correction

- Removed `RateSlew`: it incorrectly replaced the gray-position steering feed-forward with a limited yaw-rate command and made sharp curves understeer.
- GrayLine now sends the position-loop turn command directly to the wheels; the yaw-rate PID only adds a bounded damping trim. Existing PWM slew remains the physical output smoothing mechanism.
- Extended GrayLine JustFloat from 10 to 17 floats; channels 0-9 are unchanged and channels 10-16 expose the yaw-rate loop.
- IMU I2C reads now report success/failure; GrayLine automatically falls back to the pre-existing position-only controller if gyro data is invalid.
- Keil build: `0 Error(s), 0 Warning(s)`.

## 2026-07-24 - Logging management

- `log status`, `log level`, `log module`, and `log telemetry` manage text diagnostics at runtime.
- Telemetry is off by default and `motion`/`grayline` are mutually exclusive JustFloat producers.

## 2026-07-25 - Non-blocking DMA log transport and crash guard

Status: implemented and clean-build verified; target verification pending.

- UART0 text logs now use one static 160-byte DMA record on full DMA channel 2. A producer starts DMA only when the slot is idle; otherwise its complete record is dropped, so no FreeRTOS task waits for UART and records cannot interleave.
- The DMA record is kept until UART EOT, not merely DMA-complete, so its source buffer cannot change while bytes remain in the UART hardware.
- SysConfig owns the UART TX-DMA and completion-interrupt configuration. Motor encoder DMA channels remain unchanged.
- `PATH_MAX_POINTS` changed from 500 to 292. The path array releases 2496 bytes; the logger adds 166 bytes of static storage, preserving 2330 bytes net free RAM.
- Formatted log staging moved from task-stack arrays into the static DMA record. FreeRTOS stack-overflow checking is now level 2; its emergency hook reports the task name directly and resets.
- The reported boot HardFault had LR in PendSV and PC=0 after restoring a task context, which is consistent with corrupted task-stack/scheduler context. The prior large per-call log buffers were a credible trigger; interleaved UART bytes were a separate serialization symptom.
- Keil clean rebuild: 0 Error(s), 0 Warning(s).

## 2026-07-25 - Reliable UART diagnostics

- Added a second 160-byte static reliable record and a UART mutex; no log task, queue, TX FIFO ISR, or dynamic log buffer was introduced.
- `LOGE`, `LOGI_INIT`, and `LOGI_RELIABLE` serialize a whole DMA record and wait for EOT. Normal `LOGI/LOGW/LOGD` remain non-blocking, single-record DMA and may drop.
- Startup task creation uses direct blocking output inside its existing critical section. Task initialization, command replies, and stack-monitor reports use the reliable path.
- EOT IRQ is enabled before scheduler tasks begin. ERROR disables active telemetry first, then emits its text; JustFloat must be explicitly re-enabled afterward.
- Link map: ZI is 30056 bytes, a 168-byte increase for the reliable buffer/control state. `s_pathPoints` remains 3504 bytes (292 points).

## 2026-07-25 - Compact boot log and Gray stack

Status: implemented and clean-build verified; on-target validation pending.

- Reduce cold-boot output to the worker-task summary, device final identities, calibration results, encoder DMA, TFT, INS yaw, CLI, and Flash self-test result.
- Keep initialization and self-test failures as reliable ERROR records; I2C scan/probe details are DEBUG-only.
- Increase `GRAY_TASK_STACK_WORDS` and the stack monitor baseline from 128 to 154 words (+104 bytes from the FreeRTOS heap).

## 2026-07-26 - Three-mode path/gray fusion

Status: implemented and Keil rebuild verified; on-target validation pending.

- `app_path.c` remains the INS path-direction authority. The new fusion mode uses grayscale only as a local steering correction; it does not alter INS pose, yaw, or recorded path points.
- `GRAYLINE`, pure `PATH`, and `PATH_FUSION` are mutually exclusive APP drive modes. Fusion grants motor output only to `app_path`; GrayLine runs as a non-driving assist source.
- CLI and TFT provide pure GrayLine, pure Path Replay, and Fusion Start/Stop. Mode changes stop and release the previous owner before starting the next owner.

## 2026-07-26 - Fusion startup path-only fallback

Status: implemented and Keil rebuild verified; on-target validation pending.

- `path fusion start` enters continuous `app_path` tracking immediately, even when no black line has been detected.
- Fresh grayscale data remains optional: it enables the existing forward/same-direction blend; no-line, stale, conflict, or reverse operation remains path-only without stopping the vehicle.

## 2026-07-27 - UART2 incremental reconstruction

Status: paused after hardware-only stage; awaiting on-target validation.

- Rebuild starts from known-good fusion commit `f315d2f`; previous UART2/ICM worktree is preserved in Git stash `pre-uart2-rebuild-2026-07-27`.
- Stage 2 changes only UART2 hardware: PB17 TX, PB18 RX, 115200 8N1, RX FIFO threshold one byte.
- No communication task/queue exists and UART2 NVIC is deliberately not enabled. I2C0 PA0/PA1 and IMU startup remain the fusion baseline.
- Clean Keil rebuild passed: Code=108600, RO-data=17616, RW-data=508, ZI-data=30228; 0 errors, 0 warnings.

## 2026-07-27 - UART2 communication runtime restored

Status: clean-build verified; on-target verification pending.

- Hardware-only checkpoint passed, so two-entry queues, protocol task, UART2 IRQ enable, and UART0 `comm` commands are restored.
- ICM files remain the fusion baseline. `app_imu.c` and `app_init.c` explicitly disable clang size optimization.
- `-Oz` is limited to App and Algorithm groups; the target/global option is empty and Port I2C files remain default optimized.
- Heap=20 KiB, path capacity=160, and Flash/monitor startup is delayed 100 ms.
- Clean Keil rebuild: Code=94280, RO=15748, RW=508, ZI=30868; 0 errors, 0 warnings.

## 2026-07-27 - Vehicle 2 branch and protocol synchronization

Status: build verified; target link test pending.

- Published this vehicle to GitHub branch `vehicle2`; Vehicle 1 is published as branch `big-car`.
- Added mirrored root `VEHICLE_SYNC.md` as the mandatory two-vehicle protocol, task, and test record.
- The Vehicle 2 App compilation group defines `CAR_COMM_NODE_ID=2U`; Vehicle 1 uses node `0x01`.
- On target: validate `comm ping 01`, `comm send 01 10 55`, `comm recv`, ACK/retry counters, and ICM initialization across reset/power cycle.
