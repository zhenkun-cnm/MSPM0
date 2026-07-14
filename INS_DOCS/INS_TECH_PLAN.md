# INS 技术方案

## 第一版目标

输出二维位姿：

```c
typedef struct {
    float x_m;
    float y_m;
    float yaw_deg;
    float v_mps;
    float w_dps;
    float left_m;
    float right_m;
    uint32_t flags;
} INS_Pose_t;
```

## 输入

- `motor_encoder_task` 每 10ms 输出左右轮增量。
- `imu_task` 继续输出现有 `IMU_Data_t`，INS 只使用 `yaw`。

## 标定常量

- `wheel_diameter = 0.048 m`
- `wheel_base = 0.125 m`
- `M1 left counts/rev = 1054`
- `M2 right counts/rev = 985`
- `left_sign = +1`
- `right_sign = +1`

## 距离公式

```text
left_m  = left_counts  / 1054 * pi * 0.048
right_m = right_counts / 985  * pi * 0.048
ds      = (left_m + right_m) / 2
```

## 姿态融合

- 主航向使用 IMU yaw。
- 第一帧 yaw 作为零点。
- 坐标积分使用中点角：

```text
x += ds * cos(yaw_mid)
y += ds * sin(yaw_mid)
```

## Flash 日志

- 起始地址：`0x00100000`
- 日志空间：8 个 4KB sector，共 32KB。
- 记录周期：200ms。
- 单条记录：32 字节。

## 验收测试

1. Keil 编译 0 error。
2. 静止时速度接近 0。
3. 前进 1m 时 `x` 接近 1m。
4. 原地转向时 yaw 方向可识别。
5. Flash 日志写入不影响实时串口输出。

---

## 2026-07-14 实现落地记录

- 新增 `APP/inc/app_ins.h`：定义 `INS_Pose_t`、状态 flag、`g_insPoseQueue` 和 `ins_task()`。
- 新增 `APP/src/app_ins.c`：10ms 位姿积分，100ms 串口输出，200ms Flash 日志。
- 修改 `APP/inc/app_motor_encoder.h`：导出 M1/M2 标定常数、10ms 周期、`MotorOdomDelta_t` 和 `g_motorOdomQueue`。
- 修改 `APP/src/app_motor_encoder.c`：每 10ms 将左右轮增量发送到里程计队列；队列满时丢弃最旧帧并保留最新帧。
- 修改 `APP/src/app_init.c`：创建 IMU、里程计、INS 位姿队列，启动 IMU 任务和 INS 任务。
- 修改 `APP/src/app_flash.c`：取消上电擦写 `0x00000000` 的旧 Flash 测试，仅保留 JEDEC 检测。
- 修改 `APP/src/app_tft.c`：IMU 检查页面改用 `xQueuePeek`，避免消费掉 INS 也需要的最新 yaw。
- 修改 Keil `.uvprojx`：将 `app_ins.c` 加入 `App/Src` 编译列表。
- `ins_task` 上电后先等待第一帧 IMU yaw；等待期间不输出位姿、不累计编码器里程。拿到 yaw 后清零位姿并进入 ready 状态，Flash 日志再延后约 2 秒初始化。

## 第一版固定参数

```text
wheel_diameter_m = 0.048
wheel_base_m     = 0.125
M1_left_cpr      = 1054
M2_right_cpr     = 985
left_sign        = +1
right_sign       = +1
imu_yaw_sign     = +1
```

如果原地左转 90 度时串口 yaw 方向与预期相反，只改 `INS_IMU_YAW_SIGN`。

## 构建结果

- 2026-07-14 Keil rebuild：`0 Error(s), 1 Warning(s)`，Program Size `Code=47372 RO-data=6608 RW-data=52 ZI-data=26628`。
- Warning：`port_led.c` 中 `LED_PORT` 与 `ti_msp_dl_config.h` 宏重定义，为既有 warning，非 INS 新增错误。
- build.log 中 SysConfig 预构建阶段仍打印 `.metadata/product.json` 提示，但本次编译、链接、hex 生成均完成。

---

## 2026-07-14 标准化确认测试阶段

当前手推测试已经证明第一版 INS 方向正确，但测试条件不足以做最终标定。下一步不改核心算法，先按固定标准复测。

### 测试精度等级

- 当前阶段定义为“非精准确认测试”。
- 当前测试只用于判断 INS 方向是否正确、误差是否大致可接受、是否可以进入下一阶段准备。
- 当前测试不用于最终标定、算法参数精修或导航精度承诺。
- 在用户明确说“开始精准测试”之前，不根据当前手推数据直接修改轮径、轮距或左右轮 counts/rev。

### 测试文件

- `INS_DOCS/INS_STANDARD_TEST_PLAN.md`：测试方法、通过标准、失败后的判断规则。
- `INS_DOCS/INS_TEST_RECORD.md`：测试记录表，填写最终稳定后的 `[INS]` 数据。

### 通过标准

- 1m 直线：`abs(X - 1.000) < 0.05m`，`abs(Y) < 0.03m`。
- 2m 直线：`abs(X - 2.000) < 0.10m`，`abs(Y) < 0.06m`。
- 60cm 正方形回原点：最终 `abs(X) < 0.08m` 且 `abs(Y) < 0.08m`。
- 原地右转 90 度：`YAW` 在 `-90° ± 8°`。

### 进入下一阶段条件

上述项目通过后，再开始实现“惯导复位/校准命令 + 串口测试菜单 + Flash 日志读取方案”。如果未通过，先按失败类型修正轮径、左右轮 counts/rev、轮距或 IMU yaw。
