# 串口详细信息无法展示 — 修复计划

## 问题
用户说"扫描可用串口"时，只返回了端口名列表（COM1, COM2...），没有制造商、描述等详细信息。

## 根因分析
1. MCP Server `serial-monitor` 已注册到 `cline_mcp_settings.json`，但 Cline 当前会话中未连接成功
2. `serial_list` 工具输出格式较简单，只展示 `path + manufacturer`
3. `.clinerules` 触发词较少，对非精确匹配无法触发 MCP 调用

## 阶段

### 阶段 1: 增强 serial_list 输出 ✓
- serial_monitor_server.py 中 serial_list 的输出增加更多字段
- 目标：输出中包含 manufacturer、description、hwid、状态

### 阶段 2: 扩展 .clinerules 触发词 ✓
- 增加 "查看串口"、"有哪些串口"、"扫描可用串口"、"查看COM口" 等更多触发词

### 阶段 3: 通知用户重启 Cline 以重新连接 MCP Server
- MCP Server 配置存在但未连接，需要重启重新加载