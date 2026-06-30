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