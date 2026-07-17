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
