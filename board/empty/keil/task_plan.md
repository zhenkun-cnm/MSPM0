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

---

## 2026-07-14 Motor Encoder / Odometry Prep

- M1 calibrated output scale: about 1054 counts/rev.
- M2 calibrated output scale: about 985 counts/rev after App-layer software x4.
- Use the calibrated per-wheel constants for later car odometry/INS fusion.
- Verify M2 DMA continuous capture past 256 raw A-rising edges before relying on it for long runs.

---

## 2026-07-14 INS v1 implementation

- Created `INS_DOCS/` with progress, QA, technical plan, and a user-facing progress text file.
- Added `APP/inc/app_ins.h` and `APP/src/app_ins.c`.
- Added a 10ms motor odometry queue from `motor_encoder_task` to `ins_task`.
- INS v1 uses M1 left = 1054 counts/rev and M2 right = 985 counts/rev.
- Enabled IMU task startup so INS can consume latest `IMU_Data_t.yaw`.
- Disabled the old destructive Flash test at `0x00000000`; INS log region starts at `0x00100000`.
- Added `app_ins.c` to the Keil project file.
- Keil rebuild passed: `0 Error(s), 1 Warning(s)`.
- Changed TFT IMU check from `xQueueReceive` to `xQueuePeek` so INS can keep reading latest yaw.

Next validation:
- Static, straight 1m, in-place 90 degree yaw, rectangle-return, and Flash log tests.

---

## 2026-07-14 INS standardized validation stage

- Initial hand-push test is good enough to continue: about 5cm error on an approximate 100cm straight line; about 2-8cm X and 0.5-2cm Y error after an approximate 60cm square return; yaw error is nearly zero.
- Current validation is explicitly a non-precision confirmation test.
- Current data is only for checking INS direction and rough usability; it must not be used for final calibration, fine parameter tuning, or navigation accuracy promises.
- Do not directly change wheel diameter, wheelbase, or counts/rev from these hand-push results.
- Switch to precision testing only after the user explicitly says "开始精准测试".
- Do not refactor the core INS algorithm yet.
- Do not enter autonomous navigation yet.
- First run standardized validation and record results in `INS_DOCS/INS_TEST_RECORD.md`.

Validation gates:
- 1m straight x3: X error < 5cm, Y offset < 3cm.
- 2m straight x3: X error < 10cm, Y offset < 6cm.
- 60cm square return x3: final X/Y errors both < 8cm.
- In-place right turn 90 degrees x3: yaw within -90 deg +/- 8 deg.
