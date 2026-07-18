# INS 技术方案

更新时间：2026-07-15

## 当前目标

默认非精准测试已经通过，当前阶段目标是恢复并增强 INS 调试能力：

- 输出二维位姿 `x_m / y_m / yaw_deg / v_mps / w_dps`。
- 支持串口命令复位、查看状态、开关 Flash 日志、打印 Flash 日志。
- 暂不修改轮径、轮距、M1/M2 counts/rev。

## 数据流

- `motor_encoder_task` 每 10ms 发送 `MotorOdomDelta_t` 到 `g_motorOdomQueue`。
- `imu_task` 每 10ms 用 `xQueueOverwrite` 发布 `IMU_Data_t` 到 `g_imuDataQueue`。
- `ins_task` 每 10ms 消费编码器增量，读取最新 IMU yaw，输出 `INS_Pose_t` 到 `g_insPoseQueue`。
- `ins_cmd_task` 解析 UART0 文本命令，通过 `g_insCmdQueue` 发给 `ins_task`。

## 公式

```text
left_m  = M1_counts / 1054.0 * pi * 0.048
right_m = M2_counts / 985.0  * pi * 0.048
ds      = (left_m + right_m) / 2
yaw     = IMU_yaw - yaw_zero
x      += ds * cos(yaw)
y      += ds * sin(yaw)
```

如果临时读不到 IMU yaw，`ins_task` 会用左右轮差速按 `wheel_base=0.125m` 短时推算 yaw。

## 串口命令

```text
ins help      打印命令列表
ins status    打印 ready、pose、flags、Flash 状态
ins reset     清零位姿并把当前 yaw 设为新零点
ins log on    擦除并开启 INS Flash 日志
ins log off   停止写 Flash 日志
ins log print 从 Flash 扫描并打印 INS 日志
```

命令响应统一使用 `[INS_CMD]` 前缀。

## Flash 日志

日志区：

```text
start = 0x00100000
size  = 32KB
record = 32 bytes
period = 200ms
capacity = 1024 records, about 204.8s
```

记录字段：

```text
magic, flags, seq, tick_ms,
x_mm, y_mm, yaw_cdeg,
v_mms, w_cdegps,
left_delta, right_delta, yaw_source
```

`tick_ms` 是开机后毫秒时间戳，来源为 `xTaskGetTickCount() * portTICK_PERIOD_MS`。它从 FreeRTOS scheduler 启动后开始计时，基本等价于小车上电运行时间；它不是 RTC 真实日期时间。

默认不写 Flash。只有 INS Ready 后执行 `ins log on` 才会擦除并开始记录。

读取方式：

```text
ins log print
```

输出格式：

```text
[INS_LOG] #0000 boot_ms=1234 X=+0.123 Y=-0.001 YAW=+1.23 V=+0.100 W=+0.50 Ld=10 Rd=11 SRC=G F=23
```

打印命令从 `0x00100000` 开始按 32 字节记录扫描，遇到 magic 不匹配停止。如果没有记录，输出 `[INS_LOG] empty`。

## 验证标准

- Keil clean rebuild：`0 Error(s)`。
- 上电后先看到 IMU 初始化，再看到 `[INS] Ready`，不提前输出位姿。
- `ins reset` 后 `X/Y/L/R/YAW` 接近 0。
- `ins log on/off` 有明确响应，INS 输出不中断。
- `ins log print` 能打印已保存的 `[INS_LOG]` 记录，且每行包含 `boot_ms`；无记录时打印 empty。

---

## 基础运动控制 app_motion

### 任务与队列

- 新增 `APP/inc/app_motion.h`、`APP/src/app_motion.c`。
- 新增 `g_motionCmdQueue`，队列长度 `4`。
- `motion_task` 周期 `20ms`，优先级 `2`，栈 `384 words`。
- 任务读取 `g_insPoseQueue` 最新 `INS_Pose_t`，通过 `g_motorCmdQueue` 控制 TB6612。

### 串口命令

```text
motion help
motion status
motion stop
motion fwd <meters>
motion back <meters>
motion turn <-180..180>
```

命令解析仍在当前 UART 文本命令任务中完成，`motion` 命令会发送 `Motion_Command_t` 到 `g_motionCmdQueue`。

### 直行控制

- `motion fwd d`：左右轮同向前进。
- `motion back d`：左右轮同向后退。
- 起步时记录 `start_x/start_y/start_yaw`。
- 进度计算：把当前 `dx/dy` 投影到起始 yaw 方向，得到直线方向位移。
- 到达条件：`progress >= target - 0.02m`。
- yaw 修正：`yaw_error = start_yaw - current_yaw`，按比例生成左右轮小幅速度差。
- 默认速度：`25%`。
- 超时：`8s`。

### 单轮定轴转向

- `motion turn angle` 支持 `-180..+180`，`0` 拒绝。
- `angle > 0`：左转，左轮不动，右轮动。
- `angle < 0`：右转，右轮不动，左轮动。
- 到达条件：yaw 误差小于等于 `3deg`。
- 默认速度：`20%`。
- 超时：`6s`。

### 安全规则

- INS 未 Ready，或 pose flags 不满足 `INS_FLAG_IMU_VALID | INS_FLAG_YAW_ZERO_READY`，拒绝启动。
- `motion stop` 立即发送左右轮速度 0，并关闭 TB6612。
- 动作完成、超时、INS 丢失都会停车并输出 `[MOTION]` 结果。

---

## Motion PID 菜单调参

- `app_motion` 暴露运行时参数 `g_motionPid`，TFT 菜单直接编辑该结构体，调整后下一控制周期生效。
- 第一版只开放两个运动环：
  - `StraightYaw`: `Kp / Ki / Kd / TrimMax`
  - `TurnYaw`: `Kp / Ki / Kd / Deadband / PwmMin / PwmMax`
- 参数暂不保存到 W25Q64，断电恢复默认值。

菜单结构：
```text
PID
  Straight
    Kp
    Ki
    Kd
    TrimMax
  TurnYaw
    Kp
    Ki
    Kd
    Deadband
    PwmMin
    PwmMax
```

直行控制：
- 目标不是让左右 raw counts 相等，而是保持起步 yaw。
- yaw PID 输出左右 PWM 差，`TrimMax` 限制最大差值。
- 左右轮编码器标定不同：M1 左轮 `1054 counts/rev`，M2 右轮 `985 counts/rev`。如果后续做轮速内环，必须先换算成 `m/s` 再比较。

转向控制：
- `motion turn angle` 生成目标 yaw，并持续用 yaw 闭环保持。
- 误差进入 `Deadband` 后双轮刹车保持。
- 若过冲，允许反向微调回目标角度。
- 单轮转向时静止轮使用 brake，不再使用 coast。
## 导航层 v1 技术实现 - 2026-07-17

### 模块
- 新增 `APP/inc/app_nav.h`
- 新增 `APP/src/app_nav.c`
- `app_init.c` 创建 `g_navCmdQueue` 并启动 `nav_task`
- `app_ins_cmd.c` 解析 `nav ...` 串口命令
- Keil `.uvprojx` 注册 `app_nav.c`

### 状态机
```text
NAV_IDLE
NAV_TURN_TO_TARGET
NAV_DRIVE_TO_TARGET
NAV_TURN_TO_FINAL_YAW
NAV_DONE
NAV_ERROR
```

### 控制逻辑
输入目标为 `x_m / y_m / yaw_deg`。导航层读取 `INS_Pose_t`，计算：
```text
dx = target_x - x
dy = target_y - y
target_heading = atan2(dy, dx)
distance = sqrt(dx^2 + dy^2)
```

然后按三段动作执行：
```text
motion turn (target_heading - current_yaw)
motion fwd distance
motion turn (target_yaw - current_yaw)
```

### 第一版阈值
```text
position_tolerance = 0.05 m
yaw_tolerance      = 3 deg
max_leg_distance   = 5.0 m
```

### 串口命令
```text
nav help
nav status
nav stop
nav goto <x_m> <y_m> <yaw_deg>
nav square <side_m>
```

### Flash
导航层不新增 Flash 区。路径复盘继续依赖 INS 日志：
```text
boot_ms, x_mm, y_mm, yaw_cdeg, v_mms, w_cdegps, left_delta, right_delta, yaw_source
```

### 测试标准
- `nav goto 0.5 0.0 0`：能前进约 0.5m 并停止。
- `nav goto 0.0 -0.5 -90`：先右转，再前进，最终 yaw 接近 -90 度。
- `nav square 0.6`：能走完四边并回到起点附近；非精准阶段目标为回点误差小于约 10cm，yaw 误差小于约 5 度。
## 2026-07-17 NAV square 修复

### 原因
旧版 `nav square` 使用理想绝对顶点；但实车采用单轮定轴转向，转向期间 `X/Y` 会变化，因此不应假设转向只改变 `yaw`。

### 修复
- `Motion_Command_t` 增加 `cmd_id`。
- `Motion_RuntimeStatus_t` 增加 `active_cmd_id / done_cmd_id / rejected_cmd_id / last_result`。
- nav 等待 motion 时必须确认同一个 `cmd_id` 被接受并完成。
- `nav square <side_m>` 改为动作序列：
```text
FWD side -> TURN +90
FWD side -> TURN +90
FWD side -> TURN +90
FWD side -> TURN +90
```

### 串口诊断
```text
[NAV] square drive 1/4 dist=0.600
[NAV] square turn 1/4 yaw_delta=+90.0
[NAV] error: motion rejected cmd_id=...
[NAV] error: motion did not start cmd_id=...
```

### 编译
Keil clean rebuild 通过：`0 Error(s), 1 Warning(s)`。剩余 warning 是既有 `LED_PORT` 宏重定义。
## 2026-07-17 Test1 赛题固定路线

### 模块
- `APP/inc/app_test1.h`
- `APP/src/app_test1.c`
- 宏开关：`APP_TEST1_ENABLE`

### 串口命令
```text
test1 help
test1 start
test1 status
test1 stop
Test1
```

### 路线
坐标输入单位为 cm，内部转换为 m：
```text
(0.90,  0.00) -> right turn -90
(0.96, -0.90) -> right turn -90
(0.04, -0.96) -> right turn -90
(0.00,  0.00) -> right turn -90
```

### 状态机
```text
IDLE
TURN_TO_TARGET
DRIVE_TO_TARGET
RIGHT_TURN
DONE
ERROR
```

### 运动回执
Test1 直接使用 motion 的 `cmd_id` 回执机制，cmd_id 前缀为 `0x71000000`，避免和 nav 的命令混淆。

### 编译
Keil clean rebuild 通过：`0 Error(s), 1 Warning(s)`。`app_test1.c` 已参与编译。

## 2026-07-17 Turn-after-drive diagnostic chain

### Purpose
The car can reach the first straight target but may stop before the next turn. This update makes the command chain observable from UART without changing route or control parameters.

### Motion hard ACK
`app_motion.c` now prints these lines through `log_printf_internal()`:
```text
[MOTION_ACK] accept cmd_id=... type=FWD/TURN value=...
[MOTION_ACK] reject cmd_id=... type=FWD/TURN value=... reason=...
[MOTION_ACK] done cmd_id=... result=...
```

### Test1 step echo
`app_test1.c` now prints state transitions:
```text
[TEST1_STEP] drive done, enter RIGHT_TURN
[TEST1_STEP] send right turn cmd_id=... yaw_delta=-90.0
[TEST1_STEP] right turn done
```

### NAV square step echo
`app_nav.c` now prints state transitions:
```text
[NAV_STEP] square drive done, enter square turn
[NAV_STEP] send square turn cmd_id=... yaw_delta=+90.0
[NAV_STEP] square turn done
```

### Diagnosis rules
- No `send right turn`: upper state machine did not issue the next command.
- `send right turn` but no `MOTION_ACK accept type=TURN`: motion rejected or did not receive the turn command.
- `MOTION_ACK accept type=TURN` but no physical turn: inspect TB6612 and single-wheel turn motor command path.
- `accept` and `done` without physical turn: inspect yaw completion condition and IMU yaw validity.

### Build
Keil clean rebuild result: `0 Error(s), 1 Warning(s)`. The warning is the existing `LED_PORT` macro redefinition.

## 2026-07-17 Arc motion v1

### Command
```text
motion arc <radius_m> <angle_deg>
```

Examples:
```text
motion arc 0.50 90
motion arc 0.50 -90
motion arc 0.40 180
```

### Control loops
Arc v1 uses one control layer: left/right wheel-speed PID inner loops.

```text
radius + angle
  -> target_left_mps / target_right_mps
  -> wheel-speed PID
  -> left/right PWM
```

IMU yaw is only the completion condition:
```text
target_yaw = start_yaw + angle_deg
done when abs(target_yaw - actual_yaw) <= 3deg
```

There is no yaw outer-loop PID and no path lateral-error PID in v1.

### VOFA JustFloat
When `LOG_PRINT_PID_ENABLE=1`, arc mode outputs 6 JustFloat channels:
```text
Ch0 target_left_mps
Ch1 actual_left_mps
Ch2 target_right_mps
Ch3 actual_right_mps
Ch4 target_yaw_deg
Ch5 actual_yaw_deg
```

### Limits
```text
radius: 0.15m..2.00m
angle:  +/-5deg..+/-180deg
center speed: 0.10m/s
```

## 2026-07-17 Path teach/replay v1

### Commands
```text
path help
path status
path clear
path record start
path record stop
path print
path replay
path stop
```

### Behavior
- Stores up to 64 points in RAM.
- During recording, saves a point when distance changes by at least 0.05m or yaw changes by at least 5deg.
- Replay uses discrete point following: turn to target point, drive to target point, then advance.
- Replay uses motion `cmd_id` ACK/done tracking like NAV/Test1.
- Before replay, place the car back at the recorded start and run `ins reset`; recorded points are relative to that start pose.
- No Flash format change and no continuous path tracking in v1.

### Build
Keil clean rebuild passed with `0 Error(s), 1 Warning(s)`. `app_path.c` compiled. The warning is the existing `LED_PORT` macro redefinition.
