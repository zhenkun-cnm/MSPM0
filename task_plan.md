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