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

## 会话 4 - 电机编码器 M2 标定分析

**日期:** 2026-07-14  
**目标:** 根据桌面测试文本和工程代码解释 M2 每圈约 984/985 的原因，并把结论写入工作记录。

### 已完成
- [x] 读取 `C:/Users/Lenovo/Desktop/MSPM0_电机编码器_标定结果.txt`
- [x] 核对 `APP/src/app_motor_encoder.c`、`port_motor_encoder2.c/h`、`ti_msp_dl_config.c/h`
- [x] 确认 M2 当前路径为 TIMG7 捕获 A 相上升沿 + DMA 快照 B 相 + App 层软件 ×4
- [x] 修正 M2 注释中的 1:30 / 1560 过期信息，记录为 1:20 / 实测约 985
- [x] 在 `APP/inc/app_motor_encoder.h` 增加 M1/M2 标定常量
- [x] 清理 `board/empty/empty.c` 末尾裸露的测试数字

### 关键发现
- M2 每圈约 984/985 是当前软件折算后的实测标定值：原始 DMA 捕获约 246 个 A 相上升沿/输出轴转，乘 4 后约 984。
- 惯导/里程计阶段应使用每路独立标定值，而不是强行把两侧都套用 1040 理论值。
- DMA 配置的持续性仍需硬件验证，尤其是连续运行超过 256 个 A 相上升沿后是否继续写入。

### 下一步
- [x] 构建验证本次整理没有引入编译错误：Keil rebuild 通过，`0 Error(s), 1 Warning(s)`；warning 为既有 `port_led.c` / `ti_msp_dl_config.h` 的 `LED_PORT` 宏重定义。
- 后续实现小车惯导时，增加 odometry 层，用 `wheel_distance = counts / counts_per_rev * wheel_circumference` 分别处理左右轮。

---

## 会话 5 - 启用 W25Q64 Flash + 更新引脚文档

**日期:** 2026-07-14
**目标:** W25Q64 SPI 引脚改用 PB6-PB9（syscfg 已配），开启 flash 任务，串口打印验证，更新所有过期文档。

### 已完成
- [x] 确认 syscfg/HAL 已将 W25Q64 SPI1 配为 PB6(CS)/PB7(MISO)/PB8(MOSI)/PB9(SCLK)，port_flash.c 无需改
- [x] `APP/src/app_init.c`: 添加 `#include "app_flash.h"`，取消 `flash_init_task` extern 注释，在 start_task 创建 flash_init 任务
- [x] `AGENTS.md`: Flash 引脚更新为 PB6-PB9，TFT 引脚与 syscfg 对齐，IMU 改为 ICM-20608 I2C
- [x] `task_plan.md`: 更新外设引脚表（W25Q64 SPI1 + ST7735 + ICM-20608 I2C0）

### 关键发现
- IMU 已从 ICM-20948 bit-bang SPI 迁移到 ICM-20608 I2C（PA0/PA1），PB6-PB9 已完全释放给 Flash 使用
- `app_flash.c` 初始化时会串口打印 JEDEC ID + 擦写读验证结果，满足"串口打印是否正常"的需求

### 下一步
- Keil 编译验证


---

## 会话 6 - 小车 INS 第一版代码接入

**日期:** 2026-07-14
**目标:** 建立第一版二维位姿估计：编码器算距离，IMU yaw 算方向，串口与 Flash 记录输出。

### 已完成
- [x] 新建 `INS_DOCS/` 专题文档目录。
- [x] 新建 `INS_PROGRESS.md`、`INS_QA.md`、`INS_TECH_PLAN.md`。
- [x] 新建 `INS_PROGRESS_FOR_USER.txt`，用问答形式给用户直接查看当前进度。
- [x] 新增 `app_ins` 模块，输出 `x_m / y_m / yaw_deg / v_mps / w_dps`。
- [x] 编码器任务新增 10ms `MotorOdomDelta_t` 队列，供 INS 消费左右轮增量。
- [x] `app_init.c` 创建 INS/里程计队列，启动 IMU 和 INS 任务。
- [x] `app_flash.c` 停止旧的 `0x00000000` 擦写验证，避免破坏后续日志/参数。
- [x] Keil 工程文件加入 `app_ins.c`。

### 待验证
- [x] Keil rebuild：`0 Error(s), 1 Warning(s)`，warning 为既有 `LED_PORT` 宏重定义。
- [ ] 静止、直线 1m、原地 90 度、矩形路径四组上车测试。
- [ ] 根据测试结果决定是否修正 `INS_IMU_YAW_SIGN`、轮距、左右轮标定。

### 额外修正
- [x] `APP/src/app_tft.c` 的 IMU 检查页面改为 `xQueuePeek`，避免 TFT 把 IMU yaw 队列消费掉，影响 INS 读取最新姿态。

### 2026-07-14 INS 输出时序修正
- [x] `ins_task` 改为等待第一帧 IMU yaw 后才进入 ready 状态。
- [x] 等待期间不打印 `X/Y/YAW`，不累计编码器里程，避免 IMU 校准阶段污染起点。
- [x] Ready 后打印 `[INS] Ready: yaw zero=...`，之后才输出位姿日志。
- [x] Keil rebuild 通过：`0 Error(s), 1 Warning(s)`。

---

## 会话 7 - INS 手推测试判断与标准化确认计划

**日期:** 2026-07-14
**目标:** 根据初步手推测试结果判断是否进入下一阶段，并建立标准化测试记录流程。

### 用户提供的测试结果
- 直线长度约 100cm，误差约 5cm；用户说明 100cm 不一定标准，且为手推测试。
- 正方形边长约 60cm，手推回到原点；X 误差约 2cm 到 8cm，Y 误差约 0.5cm 到 2cm，偏航角几乎无误差。

### 判断
- [x] 第一版 INS 已经可用，不需要大范围返工。
- [x] 当前测试条件不足以做精确标定。
- [x] 当前阶段明确按“非精准确认测试”执行。
- [x] 当前测试数据不用于直接修改轮径、轮距或 counts/rev。
- [x] 只有用户明确说“开始精准测试”时，才切换到精准测试流程。
- [x] 先做标准化确认测试，通过后再进入“惯导复位/校准命令 + 串口测试菜单 + Flash 日志读取方案”。

### 已完成
- [x] 新增 `INS_DOCS/INS_STANDARD_TEST_PLAN.md`。
- [x] 新增 `INS_DOCS/INS_TEST_RECORD.md`。
- [x] 更新 `INS_PROGRESS.md`、`INS_PROGRESS_FOR_USER.txt`、`INS_TECH_PLAN.md`。
- [x] 在 INS 测试文档和工程总记录中增加“非精准测试”标记。

### 下一步
- [ ] 用户按记录表完成 1m/2m 直线、60cm 正方形、右转 90 度测试。
- [ ] 根据记录决定进入下一阶段或修正轮径/轮距/yaw。
