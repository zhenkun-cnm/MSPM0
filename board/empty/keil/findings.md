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
