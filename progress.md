# 进度日志 - MSPM0G3507 FreeRTOS 嵌入式系统

## 会话 1 - 项目规划初始化

**日期：** 2026-06-11  
**目标：** 理解项目结构，建立规划文件，定义后续工作

### 已完成
- [x] 加载 planning-with-files-zh 技能
- [x] 确认无现有规划文件
- [x] 分析项目目录结构
- [x] 读取 main() 启动流程
- [x] 读取 app_init.c（任务创建逻辑）
- [x] 读取 port_system.c（硬件初始化）
- [x] 读取 port_imu.c（ICM-20948 SPI 驱动，351 行核心代码）
- [x] 读取 app_imu.c（IMU 应用层任务）
- [x] 读取 dev_imu.h（Device 层 OOP 接口）
- [x] 创建 task_plan.md
- [x] 创建 findings.md
- [x] 创建 progress.md

### 关键发现
1. 项目采用四层解耦架构（HAL → Port → Device → APP）
2. Device 层使用函数指针结构体 OOP 设计
3. ICM-20948 使用软件 SPI (bit-bang)，非硬件 SPI
4. Objects/ 目录有 .o 文件，证明编译过

### 下一步
- 深入检查 Encoder/Flash/LED 模块的 Device 层接口
- 确认编译状态
- 根据用户需求制定功能扩展计划

---

## 项目规划结果

项目规划已完成。三个规划文件存储在 `c:\ti\mspm0_project\` 下：
- **task_plan.md** - 阶段规划、决策记录、模块清单
- **findings.md** - 架构分析、代码发现、硬件引脚总表  
- **progress.md** - 会话进度日志（本文件）

### 架构全貌
四层解耦架构，5 个模块全部就绪，4 个外设驱动完成。

---

## 会话 2 - 添加 ST7735 TFT 彩屏驱动

**日期：** 2026-06-22  
**目标：** 为项目添加 0.96 寸 ST7735 TFT 彩屏驱动，至少支持字符串显示

### 已完成
- [x] 确认硬件引脚配置（PA17/SCK, PA18/MOSI, PB13/RES, PB12/DC, PB11/CS, PB10/BLK）
- [x] 确认与 W25Q128 共用 SPI1 策略（CS 互斥）
- [x] 确认用户要求：GPIO 默认全低电平，初始化时需纠正为工作电平
- [x] 创建 `Device/inc/dev_tft.h` — Device 层 OOP 接口
- [x] 创建 `board/empty/port/src/port_tft.c` — Port 层驱动实现（640+ 行）
- [x] 创建 `board/empty/port/inc/port_tft.h` — Port 层头文件
- [x] 创建 `APP/inc/app_tft.h` — 应用层任务声明
- [x] 创建 `APP/src/app_tft.c` — 应用层任务实现
- [x] 更新 `task_plan.md` — 新增 TFT 模块记录
- [x] 更新 `progress.md` — 本日志

### 关键发现
1. `ti_msp_dl_config.h` 已有 LCD 引脚定义（通过 SysConfig 配置），可直接复用宏
2. SPI1 已由 W25Q64_init() 初始化，TFT 复用同一硬件 SPI
3. TFT 仅需 MOSI（单工写），不占用 MISO，但需等待并丢弃 RX FIFO 数据
4. DC 引脚控制命令/数据切换，CS 引脚控制设备选择

### 引脚电平初始化策略
用户反馈各 GPIO 默认均为低电平，初始化时纠正：

| 引脚 | 默认 | 初始化后 | 原因 |
|------|------|---------|------|
| PB13 RESET | 低 | 先低→延时→高 | 低有效复位，复位后恢复高 |
| PB12 DC | 低 | 保持低 | 低=命令模式，初始化序列从命令开始 |
| PB11 CS | 低 | 拉高 | 低有效片选，空闲不选中 |
| PB10 BLK | 低 | 拉高 | 高=背光使能，点亮屏幕 |

### 支持的功能
- `init()` — 完整初始化（GPIO + 硬件复位 + SPI + ST7735 序列）
- `fillScreen(color)` — 全屏填充
- `drawPixel(x, y, color)` — 单点绘制
- `fillRect(x, y, w, h, color)` — 矩形填充
- `drawChar(x, y, ch, color, bg)` — ASCII 8×16 字符绘制
- `printString(x, y, str, color, bg)` — 指定位置字符串
- `setCursor(x, y)` + `print(str, color, bg)` — 流式打印

### 错误记录
- Lint 错误（include file not found）：VSCode IntelliSense 缺少 Keil include path，不影响实际编译（其他 port_xxx.c 同样引用且编译通过）

### app_init.c 集成方式
在 `start_task` 中添加（约第 N 行，创建其他任务处）：

```c
#include "app_tft.h"

// 在 start_task 中创建：
BaseType_t ret = xTaskCreate(tft_task, "tft", 512, NULL, 2, NULL);
if (ret == pdPASS) {
    LOG_INFO("[INIT] tft_task created (prio=2, 512w)\r\n");
}
```

### 下一步
1. 在 Keil 工程中添加 `port_tft.c`、`app_tft.c` 编译项
2. 在 `app_init.c` 中引入并创建 `tft_task`
3. 确认编译通过并烧录测试

---

## 会话 3 - 添加独立按键任务（PA27/PB27）+ 编码器方向取反

**日期：** 2026-06-22
**目标：** 新增两个独立物理按键（PA27/PB27）的检测任务，支持短按/长按/双击，非阻塞，串口打印验证；顺便把编码器旋转方向左右对调

### 已完成
- [x] 确认 PA27/PB27 已在 syscfg/HAL 配置为输入（`BUTTON_BUTTON1_*` / `BUTTON_BUTTON2_*`，均低电平有效，PB27 外部上拉）
- [x] 创建 `Device/inc/dev_button.h` — Device 层 OOP 接口（事件含按键 ID）
- [x] 创建 `board/empty/port/inc/port_button.h` — Port 层头文件
- [x] 创建 `board/empty/port/src/port_button.c` — Port 层驱动（移植编码器按键 FSM，扩展为两路数组）
- [x] 创建 `APP/inc/app_button.h` + `APP/src/app_button.c` — 应用任务（5ms 轮询，drain 事件后 LOG_INFO）
- [x] 修改 `APP/src/app_init.c` — 注册 button_task（prio=2, 256w）
- [x] 修改 `board/empty/port/src/port_encoder.c` — `enc_decode()` 中 `dir = -dir` 实现方向取反（含菜单/position）
- [x] 修改 Keil `.uvprojx` — App/Src 加 app_button.c，Port/Src 加 port_button.c
- [x] UV4 无头编译通过：**0 Error(s), 1 Warning(s)**（warning 为既有 port_led.c LED_PORT 宏重定义，与本次无关）

### 关键设计
1. 复用 `port_encoder.c` 的按键五状态 FSM（IDLE/DEBOUNCE_DOWN/PRESSED/DEBOUNCE_UP/WAIT_DOUBLE）+ 环形队列，做成 `s_fsm[2]` 数组，每路按键各持一份独立状态
2. 事件接口 `DevButton_Event_t {id, type}`，串口打印 `BTN1: SHORT_PRESS` / `BTN2: LONG_PRESS` 可区分两键
3. 全程非阻塞：周期轮询 + FSM + 队列，无 busy-wait
4. 时间阈值复用编码器：短按≤800ms / 长按≥1000ms / 双击间隔≤250ms / 消抖 2×5ms

### 下一步（待硬件验证）
1. 烧录后串口逐项验证 BTN1/BTN2 的短按/长按/双击打印
2. 验证编码器方向：CW 现打印 LEFT、CCW 打印 RIGHT，菜单导航随之对调
3. 若 PB27 误报/无反应 → 回退到代码强制内部上拉

---

## 后续会话模板

### 会话 N - [标题]
**日期：** YYYY-MM-DD  
**目标：** [描述]

#### 已完成
- [ ] 

#### 关键发现
- 

#### 错误记录
- 

#### 下一步
-

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

- Added `app_nav` navigation layer on top of existing INS pose and motion PID primitives.
- New UART commands: `nav help`, `nav status`, `nav stop`, `nav goto <x_m> <y_m> <yaw_deg>`, `nav square <side_m>`.
- Navigation v1 uses turn-to-target, drive-to-target, turn-to-final-yaw. It intentionally does not do arcs, obstacle avoidance, or path smoothing yet.
- Flash logging remains the existing INS log area; no new W25Q64 region was added.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)` after code cleanup; remaining warning is the existing `LED_PORT` macro redefinition in `port_led.c`.
## 2026-07-17 - Fixed NAV square sequencing

- Added motion command handshake fields: `cmd_id`, `active_cmd_id`, `done_cmd_id`, `rejected_cmd_id`, `last_result`.
- NAV now waits for the exact motion `cmd_id` to be accepted and completed; rejected or non-starting motion commands put NAV into error.
- Reworked `nav square <side_m>` from ideal waypoint navigation to an explicit sequence: four forward legs and four `+90 deg` left turns.
- Keil clean rebuild passed: `0 Error(s), 1 Warning(s)`; the remaining warning is the existing `LED_PORT` macro redefinition.
## 2026-07-17 - Test1 fixed route added

- Added independent `app_test1` module guarded by `APP_TEST1_ENABLE`.
- Added UART commands: `test1 help`, `test1 start`, `test1 status`, `test1 stop`, and `Test1` alias.
- Test1 route uses cm inputs converted to meters: `(0.90,0.00)`, `(0.96,-0.90)`, `(0.04,-0.96)`, `(0.00,0.00)`, with `motion turn -90` after each point.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`; `app_test1.c` compiled.

## 2026-07-17 - Test1/nav turn-after-drive diagnostics

- Added motion hard ACK logs: `[MOTION_ACK] accept/reject/done`, printed through `log_printf_internal()` so they are not hidden by normal log suppression.
- Added Test1 step logs: `[TEST1_STEP] drive done`, `send right turn`, and `right turn done`.
- Added NAV square step logs: `[NAV_STEP] square drive done`, `send square turn`, and `square turn done`.
- No PID, wheel parameter, route coordinate, or flash format changes were made.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`; the remaining warning is the existing `LED_PORT` macro redefinition.

## 2026-07-17 - Arc motion and path teach/replay v1

- Added `motion arc <radius_m> <angle_deg>` for standard arc testing.
- Arc v1 uses only wheel-speed PID inner loops; IMU yaw is used as the stop condition, not as an outer PID loop.
- VOFA JustFloat arc output uses 6 float channels when `LOG_PRINT_PID_ENABLE=1`: target/actual left mps, target/actual right mps, target/actual yaw.
- Added `app_path` teach/replay test module with `path record start/stop`, `path print`, `path replay`, `path clear`, `path status`, and `path stop`.
- Path v1 stores up to 64 RAM points and replays them as discrete turn+drive point steps through existing motion commands.
- Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`; `app_path.c` compiled and the remaining warning is the existing `LED_PORT` macro redefinition.

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

## 2026-07-20 - Path replay reverse differential project started

- User observed that backward replay can move backward, but cannot perform backward differential turning correctly.
- Decision: treat this as a multi-step tuning project instead of making one large uncontrolled change.
- Current protective rule: preserve forward replay behavior first; introduce signed wheel PWM only where needed for reverse segments.
- PID tuning entry will be TFT (`PID -> PathTrack` later). Non-PID path tracking parameters will be UART (`path tune` later).
- Created a dedicated human-readable project status text file: `path_replay_reverse_diff_status.txt`.
- Next implementation step: convert `APP/src/app_path.c` path tracking constants into a runtime config structure, then add UART `path tune` commands.

## 2026-07-21 - Path reverse differential diagnostics

- Replaced the unverified signed-PWM implementation assumption with a diagnostic-first step: forward and reverse replay keep the same `PATH_TRACK_*` parameters and existing global-direction + positive-PWM path output.
- Added `speedtest reverse <left_pwm> <right_pwm>` to reproduce the current reverse path output, and `speedtest signed <left_pwm> <right_pwm>` as a hardware comparison only.
- Expanded `[PATH_TRACK]` output with recorded segment yaw, movement heading, segment direction error, track heading, raw/clamped trim, raw PWM, and final PWM; `[PATH_DIR]` prints immediate F/B transitions.
- Keil build passed with `0 Error(s), 0 Warning(s)`.

## 2026-07-21 - GrayLine TFT small-gain precision

- TFT float rendering now shows four decimal places for menu items with a step below `0.001`. `RateKp`, `RateKi`, and `RateKd` use a `0.0001` step, so `0.0012` is now visible and can be verified on screen.
- Pending on-target work: capture reverse and signed speedtest logs before changing any path replay control law.

## 2026-07-21 - INS command stack hardfault guard

- First reverse bench evidence: `speedtest reverse 20 12` reported `-166/-135 mm/s`, confirming the existing global-reverse differential PWM path produces unequal reverse wheel speeds.
- The next long speedtest log ended in an M0+ HardFault with `PC=0`, consistent with `ins_cmd` stack corruption during formatted logging.
- Increased `INS_CMD_TASK_STACK_WORDS` from 160 to 256 and added `stack_min_free=<words>` to `speedtest status`.
- Path replay, PathTrack values, and TB6612 control behavior are unchanged.

## 2026-07-21 - Non-blocking UART logger

- The dynamic `log_tx` queue/task was removed after it caused an immediate boot HardFault in PendSV with `pxCurrentTCB=0x07070707`.
- UART0 TX interrupt now serializes two static 128-byte records: active transmission plus newest pending. A pending replacement increments `log_overwrite` without blocking the producer.
- RX buffer is reduced to 128 bytes for the existing 80-byte command line. Stack-overflow level 2 and malloc-failed hooks now report directly through UART then reset.

## 2026-07-21 - UART asynchronous logging rollback

- Per user request, restored direct blocking UART logging and removed both asynchronous logger implementations after boot-time faults.
- Restored 256-byte RX and default disabled FreeRTOS stack/malloc hooks.
- Retained speedtest reverse/signed support, `ins_cmd` 256-word stack, and `stack_min_free` status field.
- Path replay, shared forward/reverse parameters, and TB6612 motor output remain unchanged.

## 2026-07-21 - GrayLine yaw-rate damping loop

- GrayLine now runs a 100 Hz yaw-rate damping loop using calibrated ICM gyro-Z data. It keeps the existing position PID feed-forward and adds a bounded yaw-rate correction.
- Default rate settings: `RateKp=0.0008`, `RateKi=0`, `RateKd=0`, `RateMax=180 dps`, `RateTrim=0.060 m/s`.
- Added five TFT parameters under `PID -> GrayLine`; no UART tuning command was added.
- JustFloat is now 17 channels at 50 Hz: original channels 0-9 are retained; channels 10-16 are position feed-forward, direct target rate, actual rate, rate error, rate trim, and active flag. Control remains 100 Hz because UART transmission is blocking.
- IMU read failure marks the rate inactive and safely continues the established position-only line-following behavior.
- Keil build passed with `0 Error(s), 0 Warning(s)`.

## 2026-07-21 - GrayLine direct-turn correction

- Removed `RateSlew`: it incorrectly replaced the gray-position steering feed-forward with a limited yaw-rate command and made sharp curves understeer.
- GrayLine now sends the position-loop turn command directly to the wheels; the yaw-rate PID only adds a bounded damping trim. Existing PWM slew remains the physical output smoothing mechanism.

## 2026-07-21 - GrayLine feedback sign and encoder-spike correction

- Initial JustFloat correlation was not sufficient to override physical steering direction. GrayLine retains IMU Z-axis sign `+1`; physical on-target behavior is the source of truth.
- GrayLine wheel-speed PID now rejects a frame if either encoder-derived speed exceeds `0.80 m/s`, while advancing its encoder reference so the same corrupt counts are not reused.

## 2026-07-24 - Runtime log management

- Replaced legacy text log macros with runtime level/module filtering: ERROR, WARN, INFO, and DEBUG across SYS, CLI, INS, MOTION, NAV, PATH, TEST1, GRAY, GRAYLINE, MOTOR, ENCODER, IMU, MAG, FLASH, TFT, BUTTON, and LED.
- Added UART commands: `log help`, `log status`, `log level`, `log module`, and `log telemetry`.
- JustFloat is disabled by default and explicitly selects either `motion` or `grayline`; selected telemetry suppresses text output to keep the UART protocol exclusive.
- Keil rebuild currently stops in the existing SysConfig pre-build command because `C:\.metadata\product.json` is missing; source compilation therefore still needs a machine with the SysConfig profile restored.

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

## 2026-07-26 - Three-mode path/gray fusion

- [x] Added `app_drive_mode` as the exclusive APP control-mode coordinator: `IDLE`, `GRAYLINE`, `PATH`, and `PATH_FUSION`.
- [x] Added `path fusion start|stop|status`, TFT `INS -> Fusion Start/Fusion Stop`, and `PID -> PathFusion` runtime parameters.
- [x] Made GrayLine a non-driving 100 Hz path-assist source during fusion; `app_path` owns the fused motor PWM.
- [x] Keil clean rebuild: 0 errors, 0 warnings (Code=108792, RO=17680, RW=512, ZI=30080).
- [ ] On target: validate gray → path → fusion → stop → gray transitions, conflict fallback, stale-line fallback, reverse path fallback, and TFT tuning.

## 2026-07-26 - Fusion startup path-only fallback

- [x] Removed the fresh-black-line startup wait and timeout error from `app_path`.
- [x] `path fusion start` now immediately starts path replay with `fusion_mode=STALE`; valid gray samples can join dynamically.
- [x] Keil clean rebuild: 0 errors, 0 warnings (Code=108600, RO=17660, RW=512, ZI=30080).
- [ ] On target: start Fusion with no black line and confirm immediate `REPLAY_TRACK` motor output; then introduce a valid line and confirm `BLEND` only when directions agree.
- Note: one initial inspection command had a PowerShell `${variable}:` interpolation parse error; the retry succeeded and no source files were changed by that failed command.

## 2026-07-27 - UART2 hardware-only reconstruction checkpoint

- [x] Preserved the previous UART2/ICM investigation worktree in Git stash `pre-uart2-rebuild-2026-07-27` and created `codex/uart2-rebuild-from-fusion` from known-good fusion commit `f315d2f`.
- [x] Regenerated SysConfig for UART2: PB17 TX, PB18 RX, 115200 8N1, RX FIFO one-byte threshold, RX peripheral interrupt source.
- [x] Added the Device/Port UART2 transport files and Keil source registration; no APP communication code, task, queue, or UART2 NVIC enable is present.
- [x] Keil clean rebuild: 0 errors, 0 warnings (Code=108600, RO=17616, RW=508, ZI=30228).
- [ ] On target: test download, MCU reset, and full power-cycle ICM initialization before proceeding to the communication runtime stage.

## 2026-07-27 - UART2 communication runtime

- [x] Hardware-only UART2 checkpoint passed on target.
- [x] Restored framed two-vehicle protocol, two-entry TX/RX queues, ACK/retry handling, UART2 RX interrupt enable from the communication task, stack monitor registration, and UART0 `comm` commands.
- [x] Retained normal fusion ICM/I2C files; `app_imu.c` and `app_init.c` are excluded from clang size optimization.
- [x] Restored 20 KiB heap, 160-point path capacity, and delayed Flash/monitor startup to avoid task-creation failures.
- [x] Keil clean rebuild: 0 errors, 0 warnings (Code=94280, RO=15748, RW=508, ZI=30868).
- [ ] On target: test ICM after download/reset/power cycle; then test bidirectional ping, send/receive, ACK retry, and `comm status`.

## 2026-07-27 - Vehicle 2 publication record

- [x] Configured the Vehicle 2 App build with `CAR_COMM_NODE_ID=2U`.
- [x] Added identical `VEHICLE_SYNC.md` records to both vehicle branches and pushed `vehicle2` with `big-car` as its peer branch.
- [x] Verified the Vehicle 2 Keil build: 0 errors, 0 warnings (Code=94280, RO=15748, RW=508, ZI=30868).
- [ ] On target: validate the shared UART2 protocol with Vehicle 1 and record `comm status` counters in `VEHICLE_SYNC.md`.
