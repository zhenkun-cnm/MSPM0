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
