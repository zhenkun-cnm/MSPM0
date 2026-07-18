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
