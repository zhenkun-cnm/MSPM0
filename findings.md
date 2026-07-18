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
