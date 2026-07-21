# 项目规划 - MSPM0G3507 FreeRTOS 嵌入式系统

## 项目概述

**目标：** MSPM0G3507 微控制器上的 FreeRTOS 多传感器嵌入式系统，采用四层解耦架构设计。

**目标硬件：** TI MSPM0G3507 (ARM Cortex-M0+, 80MHz)  
**开发环境：** Keil MDK (uVision)  
**实时操作系统：** FreeRTOS (Middleware/FreeRTOS)  
**项目路径：** `c:\ti\mspm0_project\board\empty\keil`

---

## 四层解耦架构

```
┌─────────────────────────────────────┐
│          APP 层（应用逻辑）          │
│  app_led.c / app_encoder.c          │
│  app_flash.c / app_imu.c            │
│  app_tft.c / app_init.c             │
├─────────────────────────────────────┤
│        Device 层（设备抽象）         │
│  dev_flash.h / dev_imu.h / dev_led.h│
│  dev_encoder.h / dev_tft.h          │
│  OOP 风格接口契约                    │
├─────────────────────────────────────┤
│         Port 层（硬件驱动）          │
│  port_system.c / port_imu.c         │
│  port_flash.c / port_encoder.c      │
│  port_led.c / port_log.c            │
│  port_tft.c                         │
│  寄存器操作 & TI DriverLib 调用      │
├─────────────────────────────────────┤
│         HAL 层（TI DriverLib）       │
│  ti_msp_dl_config.c/h               │
│  SYSCFG 自动生成的外设配置           │
└─────────────────────────────────────┘
```

**分层规则：**
- APP 层：仅调用 Device 接口 + FreeRTOS API，禁止直接操作寄存器
- Device 层：纯 `.h` 头文件，定义 OOP 接口契约（函数指针结构体）
- Port 层：实现 Device 层的接口，操作硬件寄存器，可以引入 `ti_msp_dl_config.h`
- HAL 层：TI SYSCFG 工具自动生成的外设初始化代码

---

## 当前模块清单

| 模块 | Device 接口 | Port 实现 | App 任务 | 状态 |
|------|------------|-----------|---------|------|
| **System** | - | port_system.c | app_init.c (start_task) | ✅ 完成 |
| **LED** | dev_led.h | port_led.c | app_led.c (led_task) | ✅ 完成 |
| **Encoder** | (待确认) | port_encoder.c | app_encoder.c (encoder_task) | ⚠️ 需验证 |
| **Flash** | dev_flash.h | port_flash.c | app_flash.c (flash_init_task) | ⚠️ 需验证 |
| **IMU** | dev_imu.h | port_imu.c | app_imu.c (imu_task) | ✅ 完成 |
| **TFT** | dev_tft.h | port_tft.c | app_tft.c (tft_task) | ✅ 新增 |
| **Button** | dev_button.h | port_button.c | app_button.c (button_task) | ✅ 新增 |
| **Log** | - | port_log.c | - | ✅ 完成 |

---

## 任务概览

| 任务名 | 优先级 | 栈大小 | 周期 | 功能 |
|--------|--------|--------|------|------|
| start_task | 1 | 128w | 一次性 | 创建所有子任务后删除自身 |
| led_task | 2 | 128w | 周期性 | LED 闪烁 |
| encoder_task | 2 | 256w | 5ms 轮询 | 编码器检测 |
| flash_init | 1 | 256w | 一次性 | Flash JEDEC ID 验证后删除 |
| imu_task | 2 | 384w | 1000ms | ICM-20948 6 轴数据采集 |
| tft_task | 2 | 512w | 1000ms | ST7735 TFT 字符串显示刷新 |
| button_task | 2 | 256w | 5ms 轮询 | PA27/PB27 独立按键短按/长按/双击 |

---

## 外部设备与引脚

### ICM-20948 (软件 SPI)
| 引脚 | 功能 |
|------|------|
| PB9  | SCK  |
| PB8  | MOSI |
| PB7  | MISO |
| PB6  | CS   |

### ST7735 TFT (SPI1 硬件，与 W25Q128 共用)
| 引脚 | 功能 | 初始电平 | 工作电平 |
|------|------|---------|---------|
| PA17 | SCK  | - | SPI1 |
| PA18 | MOSI | - | SPI1 |
| PB13 | RESET | 低 | 高（低有效复位后恢复高） |
| PB12 | DC   | 低 | 低=命令 / 高=数据 |
| PB11 | CS   | 低 | 高（空闲不选中） |
| PB10 | BLK  | 低 | 高（背光使能） |

---

## 阶段规划

### 阶段 1：项目基线确认 ✅
- [x] 确认四层架构设计
- [x] 确认系统启动流程
- [ ] 验证所有模块编译通过

### 阶段 2：模块完整性验证
- [ ] 确认 Encoder 模块 Device 层接口是否存在
- [ ] 确认 Flash 模块 Device 层接口完整性
- [ ] 确认 LED 模块接口完整性

### 阶段 3：功能验证计划
- [ ] 制定各模块单元测试方案
- [ ] 确认硬件连接是否正确
- [ ] 制定调试策略

### 阶段 4：待解决问题
- [x] TFT ST7735 驱动（2026-06-22 完成）
- [ ] 是否需要添加新功能？
- [ ] Flash 模块是否需要持久化存储功能？
- [ ] TFT 是否需要在 app_init.c 中创建 tft_task
- [ ] Keil 工程是否需添加 port_tft.c / app_tft.c 编译项

---

## 遇到的错误

| 错误 | 尝试次数 | 解决方案 | 状态 |
|------|---------|---------|------|
| - | - | - | - |

---

## 决策记录

| 日期 | 决策 | 原因 |
|------|------|------|
| - | 采用四层解耦架构 | 硬件/业务分离，便于移植和维护 |
| - | Device 层纯头文件 OOP 接口 | 编译期间接口校验，运行零开销 |
| - | Port 层软件 SPI (bit-bang) | 避免 TI DriverLib SPI API 复杂性 |
| 2026-06-22 | TFT 使用 SPI1 硬件（与 W25Q128 共用） | PA17/PA18 已配 SPI1，CS 独立即可 |

---

## 下一步行动

1. 在 Keil 工程中添加 port_tft.c / app_tft.c 编译项
2. 在 app_init.c 中引入并创建 tft_task
3. 确认编译通过并烧录测试
## 2026-07-17 - Current task: NAV v1

Status: implemented and build-verified.

Completed:
- Add `APP/inc/app_nav.h` and `APP/src/app_nav.c`.
- Add `g_navCmdQueue`, `nav_task`, and Keil project registration.
- Add UART commands for `nav help/status/stop/goto/square`.
- Use INS pose plus motion primitives for point navigation.
- Reuse INS Flash logging for navigation replay.

Next validation:
- `ins reset`
- `nav goto 0.5 0.0 0`
- `ins reset`
- `nav goto 0.0 -0.5 -90`
- `ins reset`
- `nav square 0.6`

Do not start arcs or obstacle avoidance until NAV v1 point/square tests are observed on the car.
## 2026-07-17 - NAV square fix

Status: implemented and build-verified.

Completed:
- Motion command/result handshake with `cmd_id`.
- NAV waits for exact accepted/done command instead of only checking IDLE.
- `nav square` now runs 4 forward legs and 4 left turns.

Next validation:
- `ins reset`
- `nav square 0.6`
- Confirm serial logs show `square drive 1/4`, `square turn 1/4`, through `square turn 4/4`.
- If a step fails, use `nav status` and `motion status` to inspect `wait/active/done/rejected/result`.
## 2026-07-17 - Test1 fixed competition route

Status: implemented and build-verified.

Completed:
- Added `APP/inc/app_test1.h` and `APP/src/app_test1.c`.
- Added `APP_TEST1_ENABLE` macro.
- Added UART `test1` commands and `Test1` alias.
- Registered `app_test1.c` in Keil project files.

Next validation:
- `ins reset`
- `ins log on`
- `test1 start`
- Use `test1 status` if a step stops or errors.

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
- Arc v1 uses wheel-speed PID only, plus IMU yaw completion.
- Added VOFA JustFloat 6-channel output for arc PID tuning under `LOG_PRINT_PID_ENABLE`.
- Added `app_path` RAM teach/replay module for irregular arcs.
- Registered `app_path.c` in Keil project files.

Next validation:
- `ins reset`
- `ins log on`
- `motion arc 0.50 90`
- `ins log off`
- `ins log print`
- Repeat with `motion arc 0.50 -90`.
- For irregular path: `path record start`, push path, `path record stop`, `path print`, move car back to the recorded start, `ins reset`, `path replay`.

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

## 2026-07-20 - Path replay reverse differential tuning

Status: planning recorded; implementation will proceed step by step.

Goal:
- Make `path replay` reproduce pushed backward curves, not only straight backward motion.
- Preserve working forward replay behavior as much as possible.
- Add tunable parameters before aggressive control changes so each test result can be compared.

Current facts:
- INS odometry already supports signed backward displacement.
- TB6612 motor task already supports independent signed wheel commands: `MOTOR_CMD_LEFT_SIGNED_SPEED` and `MOTOR_CMD_RIGHT_SIGNED_SPEED`.
- Current path tracking has reverse segment detection and `dir=F/B` logs, but reverse tracking still uses a global reverse direction plus positive left/right PWM.

Implementation phases:
1. [x] Record the engineering plan and create a human-readable project status `.txt`.
2. [ ] Convert path tracking constants into a runtime tuning config structure.
3. [ ] Add UART `path tune` commands for non-PID parameters: lookahead, base forward/back PWM, trim max, PWM min/max/slew, done distance.
4. [ ] Add TFT `PID -> PathTrack` page for Kp/Ki/Kd only.
5. [ ] Change reverse path tracking to use left/right signed PWM while keeping forward replay on the existing output path initially.
6. [ ] Run controlled tests: forward straight, backward straight, backward curve, mixed forward/back path.
7. [ ] Tune parameters from serial logs and record each effective setting.

Validation commands:
- `path tune`
- `ins reset`
- `path clear`
- `path record start`
- Push a forward or backward test path.
- `path record stop`
- `path print`
- Return car to recorded start pose.
- `ins reset`
- `path replay`

Success criteria:
- Forward path still logs `dir=F` and behaves like the current known-good replay.
- Backward straight logs `dir=B` and signed negative PWM after the signed-PWM phase.
- Backward curve logs unequal negative left/right PWM and physically turns while reversing.
- Mixed path changes direction without a large jump, spin, or early finish.
