# Motor 2 编码器 TIMG7 输入捕获修复计划

## 问题
TIMG7 RIS 从 0x0023 变成 0x0033 后不再更新 → 后续边沿未被捕获。
DMA 也未被触发（DMA-rem=256 始终不变）。

## 根因假设（按优先级）
1. **TIMG7 捕获模式未正确启动** — `DL_TimerG_startCounter()` 对 Capture 模式可能不够，需要用 `DL_TimerG_startTimer()` 或清除捕获锁存
2. **CC0 锁存后需要软件读走才能再次捕获** — Edge-Time 模式下每次捕获后需读 CC0 寄存器或清除标志
3. **LOAD_VALUE=63999 导致计数器在捕获前溢出** — 周期太短，CNT 频繁回绕干扰捕获

## 阶段

### 阶段 1: 验证 CC0 寄存器 🔄
- ✅ 在诊断日志中加入 CC0 读数 (DL_TimerG_getCaptureCompareValue)
- ✅ 同时打印 CTR 进行对比
- 🔄 等待用户烧录验证
- 若 CC0 不变 → 捕获根本没工作，问题在定时器配置
- 若 CC0 随边沿更新 → 捕获正常，问题在 Event Fabric 订阅

### 阶段 2: 修复捕获配置
- 若 CC0 不更新：
  - 尝试用 `DL_TimerG_clearInterruptStatus()` 或读 CC0 解锁捕获锁存
  - 换用 `DL_TimerG_startTimer()` 替代 `startCounter()`
  - 增大 LOAD_VALUE 为 65535 减少溢出干扰
- 若 CC0 更新但 DMA 不触发：
  - 在 MSPM0 设备头文件中找到 GEN_EVENT 外设基址
  - 写 `SUBSCRIBE_CFG[1] = 1` 建立事件路由

### 阶段 3: 验证完整链路
- 旋转编码器 → CC0 变化 → DMA-rem 递减 → g_motor2PulseCount 非零