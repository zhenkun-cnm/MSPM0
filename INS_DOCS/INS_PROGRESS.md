# INS 进度记录

更新时间：2026-07-15

## 当前状态

当前按“非精准测试数据正常”处理，第一版 INS 已恢复并进入调试功能阶段。

已完成：

- 恢复 `app_ins`：10ms 轮速里程计 + IMU yaw 输出二维位姿。
- 恢复 IMU yaw 队列：`IMU_Data_t` 每 10ms 发布一次最新姿态。
- 增加串口命令：`ins help/status/reset/log on/log off`。
- 增加 Flash 日志打印命令：`ins log print`。
- Flash 日志默认关闭，只有 `ins log on` 后才擦除并写入 INS 日志区。
- Keil clean rebuild 通过：`0 Error(s), 1 Warning(s)`。
- 新增 `ins log print` 后再次 clean rebuild 通过：`0 Error(s), 1 Warning(s)`。
- Flash 日志中的 `tick_ms` 已明确为 `boot_ms`：开机后毫秒时间戳。
- 修改 `boot_ms` 打印后 Keil clean rebuild 通过：`0 Error(s), 1 Warning(s)`。

## 问答

### 问：现在下一步已经开始了吗？

答：是。由于非精准测试数据按正常处理，现在不再卡在精准测试，已经进入“调试命令 + 日志复盘”阶段。

### 问：现在有没有修改轮径、轮距、counts/rev？

答：没有。当前仍使用：

- 轮径：`48 mm`
- 轮距：`125 mm`
- M1 左轮：`1054 counts/rev`
- M2 右轮：`985 counts/rev`

### 问：串口命令有哪些？

答：

```text
ins help
ins status
ins reset
ins log on
ins log off
ins log print
```

### 问：`ins reset` 做什么？

答：清零 `X/Y/L/R/V/W`，并把当前 IMU yaw 当作新的 0 度方向。用于每次测试前重新设定起点。

### 问：Flash 现在存什么？

答：Flash 存 INS 位姿日志，每条 32 字节：

```text
magic / flags / seq / tick_ms / x_mm / y_mm / yaw_cdeg /
v_mms / w_cdegps / left_delta / right_delta / yaw_source
```

其中 `tick_ms` 打印时显示为 `boot_ms`，代表从开机/调度器启动到当前日志的毫秒数，不是真实日期时间。

### 问：什么时候写 Flash？

答：默认不写。只有 `[INS] Ready` 后发送 `ins log on`，才会擦除 `0x00100000` 开始的 32KB INS 日志区，然后每 200ms 写一条记录。发送 `ins log off` 后停止写。

### 问：Flash 里的日志怎么看？

答：发送：

```text
ins log print
```

它会从 `0x00100000` 开始扫描 INS 日志区，把每条有效记录打印成 `[INS_LOG]`。如果没有记录，会打印 `[INS_LOG] empty`。

每条 `[INS_LOG]` 会包含 `boot_ms=...`，用来判断这条记录发生在开机后多久。

### 问：为什么不默认一直写？

答：减少 Flash 磨损，也避免上电初始化阶段误写无意义数据。需要复盘轨迹时手动开启。

### 问：现在怎么测试？

答：

1. 上电等待 `[INS] Ready`。
2. 输入 `ins reset`。
3. 需要日志时输入 `ins log on`。
4. 推车或跑车。
5. 结束后输入 `ins log off`。
6. 输入 `ins log print` 查看 Flash 保存的轨迹记录。
7. 看串口 `[INS] X/Y/YAW/V/W/L/R/SRC/F` 判断当前位姿。

---

## 2026-07-15 基础运动控制阶段

### 问：现在进入哪一步？

答：已经从“只看 INS 位姿”进入“用 INS 位姿控制小车基础动作”的阶段。当前实现第一版 `app_motion`，用于串口命令控制直行和单轮定轴转向。

### 问：新增了哪些命令？

答：

```text
motion help
motion status
motion stop
motion fwd 0.5
motion back 0.5
motion turn 45
motion turn -90
```

### 问：转向是不是只能 90 度？

答：不是。`motion turn angle` 支持 `-180` 到 `+180` 度，正数左转，负数右转，`0` 度拒绝执行。

### 问：单轮转向规则是什么？

答：

- `motion turn 45`：左转，左轮不动，右轮动。
- `motion turn -90`：右转，右轮不动，左轮动。

这是按你的要求实现的“一个轮子不动，另一个轮子动”的定轴转向。

### 问：直行如何判断到达？

答：运动开始时记录起点 `X/Y/YAW`，之后用当前 INS 位移投影到起始车头方向上，达到目标距离后自动停车。直行时还会用 yaw 误差做小幅左右轮速度修正。

### 问：安全保护有哪些？

答：INS 没 Ready 时拒绝启动；`motion stop` 随时停车；直行默认 8 秒超时；转向默认 6 秒超时；完成、超时、异常都会停车并打印 `[MOTION]` 结果。

### 编译验证

Keil clean rebuild 已通过：`0 Error(s), 1 Warning(s)`。唯一 warning 仍是原有 `LED_PORT` 宏重定义。`map` 文件已确认 `motion_task` 和 `g_motionCmdQueue` 链接进固件。
## 2026-07-17 导航层 v1

**现在惯导做到哪一步？**  
已经进入导航层 v1：`app_nav` 会读取 INS 位姿，并通过现有 `motion` 动作完成“转向对准目标 -> 直行到目标 -> 转向到最终角度”。

**新增了什么命令？**  
- `nav help`
- `nav status`
- `nav stop`
- `nav goto <x_m> <y_m> <yaw_deg>`
- `nav square <side_m>`

**为什么第一版不用圆弧？**  
当前最终目标是先让小车能按 INS 位姿到达目标点。分段式导航更容易定位误差来源：转向误差、直行误差、最终角度误差可以分开看。

**下一步怎么测？**  
先做非精准测试：
```text
ins reset
nav goto 0.5 0.0 0
ins reset
nav goto 0.0 -0.5 -90
ins reset
nav square 0.6
```

**Flash 怎么记录？**  
继续复用 INS 日志，不新增 Flash 区。建议测试时使用：
```text
ins reset
ins log on
nav square 0.6
ins log off
ins log print
```
## 2026-07-17 修复 nav square 只走一条直线

**为什么之前可能只走一条直线？**  
旧的 nav 和 motion 之间没有明确命令回执，nav 可能只看到 motion 是 `IDLE`，就误判下一步动作已经完成。

**这次怎么修？**  
- motion 命令增加 `cmd_id`。
- motion 运行状态增加接受、完成、拒绝回执。
- nav 必须等同一个 `cmd_id` 完成，才能进入下一步。
- `nav square` 从理想顶点导航改成动作序列：四次直行 + 四次左转 90 度。

**现在怎么判断修好了？**  
运行：
```text
ins reset
nav square 0.6
```
串口应依次看到 `square drive 1/4`、`square turn 1/4`，直到 `square turn 4/4` 和 `square done`。
## 2026-07-17 Test1 固定路线

**现在新增了什么？**  
新增 `app_test1`，用于赛题固定路线。串口发送 `test1 start` 或 `Test1` 启动。

**路线是什么？**  
```text
X=90cm  Y=0cm    -> 右转90度
X=96cm  Y=-90cm  -> 右转90度
X=4cm   Y=-96cm  -> 右转90度
X=0cm   Y=0cm    -> 右转90度
```

**如何关闭节省内存？**  
在 `APP/inc/app_test1.h` 中将：
```c
#define APP_TEST1_ENABLE 1
```
改为：
```c
#define APP_TEST1_ENABLE 0
```

## 2026-07-17 Test1/nav turn-after-drive diagnostics

**现象**  
`test1 start` 和之前的 `nav square 0.6` 类似，实车表现为走完第一段直线后停止，不继续转弯。

**本次处理**  
- `app_motion.c` 增加硬回显 `[MOTION_ACK] accept/reject/done`，使用 `log_printf_internal()`，不受普通日志抑制影响。
- `app_test1.c` 增加 `[TEST1_STEP]`，直行完成、发送右转、右转完成都会打印。
- `app_nav.c` 增加 `[NAV_STEP]`，`nav square` 的直行完成、发送转弯、转弯完成都会打印。
- 不修改 PID、轮径、轮距、M1/M2 counts/rev，也不修改 Test1 路线坐标。

**下一次实车测试判据**  
发送：
```text
ins reset
test1 start
```

第一段直线完成后应看到：
```text
[TEST1_STEP] drive done, enter RIGHT_TURN
[TEST1_STEP] send right turn ...
[MOTION_ACK] accept ... type=TURN
```

如果仍不转弯：
- 没有 `[TEST1_STEP] send right turn`：Test1 状态机没有进入右转发送。
- 有 `[TEST1_STEP] send right turn` 但没有 `[MOTION_ACK] accept type=TURN`：motion 未接受或拒绝转弯命令。
- 有 `[MOTION_ACK] accept type=TURN` 但电机不动：检查 TB6612 单轮转向、电机方向、PWM、brake/coast。
- 有 accept 和 done 但车没转：检查 yaw 完成条件或 IMU yaw 数据。

**编译结果**  
Keil clean rebuild 通过：`0 Error(s), 1 Warning(s)`。剩余 warning 是既有 `LED_PORT` 宏重复定义。
## 2026-07-17 Arc motion and path teach/replay v1

**现在新增了什么？**  
新增标准圆弧命令和不规则路径手推记录/复现模块。

**标准圆弧怎么测？**
```text
ins reset
ins log on
motion arc 0.50 90
ins log off
ins log print
```

右圆弧：
```text
ins reset
ins log on
motion arc 0.50 -90
ins log off
ins log print
```

**圆弧 PID 是几环？**  
第一版只有轮速内环：左轮速度 PID + 右轮速度 PID。IMU yaw 只用来判断角度到达，不作为 yaw 外环 PID。

**VOFA 输出是什么？**  
当 `LOG_PRINT_PID_ENABLE=1` 时，圆弧运行期间输出 6 通道 JustFloat：
```text
target_left_mps
actual_left_mps
target_right_mps
actual_right_mps
target_yaw_deg
actual_yaw_deg
```

**不规则圆弧怎么测？**
```text
ins reset
path record start
手推一段弧线或 S 形
path record stop
path print
ins reset
path replay
```

**编译结果**  
Keil clean rebuild 通过：`0 Error(s), 1 Warning(s)`。`app_path.c` 已参与编译，唯一 warning 仍是已有 `LED_PORT` 宏重复定义。
