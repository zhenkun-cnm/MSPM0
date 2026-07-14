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

## 2026-07-14 电机编码器标定发现

### 结论
- M1 使用 TIMG8 QEI 硬件正交解码，13 线编码器 × 4 倍频 × 1:20 减速比，理论为 1040 counts/output rev，桌面标定文本记录实测约 1054。
- M2 使用 TIMG7 Edge-Time Capture + DMA，只在 PA28/A 相上升沿捕获 GPIOA 快照，因此 Port 层原始计数是 1x；App 层通过 `g_motor2PulseCount = delta * 4` 折算到 4x 标尺。
- M2 实测 984~988 counts/output rev，稳定值约 985。这个数不是 1:30 的 1560，也不是纯理论 1040，而是当前硬件、边沿捕获链路和软件 ×4 折算后的标定常数。

### 对惯导/里程计的影响
- 后续小车惯导融合时，左右轮里程换算应优先使用实测常数：M1 约 1054 counts/rev，M2 约 985 counts/rev。
- M2 不建议再用 1:30 注释或 1560 参与换算；那会让右轮距离被低估约 36.9%。
- `port_motor_encoder2.c` 依赖 DMA 持续搬运快照；当前生成配置中 DMA 为 repeat-single 传输模式，但 `extendedMode` 是 NORMAL，需上车前验证连续高速计数是否会在 256 个捕获后停止。

### 本次代码整理
- `APP/inc/app_motor_encoder.h` 新增 M1/M2 每圈标定常量和 M2 软件 ×4 常量。
- `APP/src/app_motor_encoder.c` 将 M2 的硬编码 `* 4` 改为 `MOTOR2_ENCODER_SOFTWARE_SCALE`。
- 更新 M2 注释为 1:20、实测约 985 counts/output rev。
- 移除 `board/empty/empty.c` 末尾裸露的标定数字，避免 C 源文件编译失败。

---

## 2026-07-14 INS v1 实现发现

- 第一版 INS 使用轮速里程计负责距离、现有 IMU yaw 负责方向；不使用加速度双积分计算位置。
- M1 按左轮处理，使用 1054 counts/rev；M2 按右轮处理，使用 985 counts/rev。
- 前进时左右轮脉冲均按正数处理，第一版 `left_sign = +1`、`right_sign = +1`。
- 原 Flash 自检会上电擦写 `0x00000000`，在引入日志/参数保存后风险较高，已改为只读 JEDEC ID。
- INS Flash 调试日志从 `0x00100000` 开始，当前预留 8 个 4KB sector，共 32KB。
- TFT 的 IMU 检查页面不能用 `xQueueReceive` 消费 IMU 队列，否则 INS 可能读不到最新 yaw；已改为 `xQueuePeek`。
- 剩余硬件风险：需要实车验证 yaw 正负方向、M2 DMA 长时间连续计数，以及 SPI 共享设备之间是否需要互斥。

---

## 2026-07-14 INS 初步手推测试发现

- 约 100cm 直线手推误差约 5cm，考虑人工手推与测距不标准，暂判定为可接受但不能用于精确标定。
- 约 60cm 正方形回原点，X 误差约 2-8cm，Y 误差约 0.5-2cm，偏航角几乎无误差，说明当前“轮速距离 + IMU yaw 方向”融合方向正确。
- 不建议直接进入自动导航；先进行标准化确认测试。
- 标准化测试通过后再进入复位/校准命令、串口测试菜单、Flash 日志读取方案。

### 测试精度等级补充
- 当前所有手推测试归类为非精准确认测试。
- 非精准确认测试只能判断 INS 是否大体可用，不能直接用于精确修正轮径、轮距或 counts/rev。
- 精准测试需要用户后续明确提出“开始精准测试”，并另行制定更严格的场地、测距、路径和记录流程。
