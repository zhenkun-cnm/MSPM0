# 进度日志

## 2026-06-11

### 诊断阶段
- 用户反馈：说"扫描可用串口"后只返回端口名列表，没有详细信息
- 根因定位：MCP Server `serial-monitor` 已在 `cline_mcp_settings.json` 中注册，但 Cline 当前会话中未建立连接
- 降级方案（`GetPortNames()`）仅返回端口名，没有制造商/描述等详细信息
- 验证：pyserial 3.5 可正常工作，返回 16 个 COM 口的完整信息（制造商、描述、HWID等）

### 修复措施
1. ✅ 增强 `serial_monitor_server.py` 的 `list_serial_ports()` 函数：
   - 新增 VID/PID 提取（从 HWID 中解析）
   - 新增 serial_number、location 字段
   - 输出格式从 `COM53 (Microsoft)` 改为带分隔线的多行详细信息

2. ✅ 扩展 `.clinerules` 触发词：
   - 新增「查看串口」「有哪些串口」「查看COM口」「列出串口」「显示串口」「查看现在有那些串口」
   - 这些词也会触发 MCP serial_list 工具调用

3. ✅ MCP 配置检查（`cline_mcp_settings.json`）：
   - serial-monitor 已正确配置 → Python314 + serial_monitor_server.py
   - `disabled: false`，配置无误

### 剩余步骤
- 用户需要重启 Cline VS Code 窗口，让 MCP Server 重新建立连接

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
