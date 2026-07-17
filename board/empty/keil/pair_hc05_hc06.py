"""
HC-05 (COM53) ↔ HC-06 (COM16) 蓝牙配对脚本
============================================
流程:
  1. 连接 HC-06 (COM16), 读取其蓝牙 MAC 地址
  2. 连接 HC-05 (COM53), 设置为主机模式
  3. 将 HC-05 绑定到 HC-06 的地址
  4. 重启 HC-05, 自动连接 HC-06
"""

import serial
import time
import re
import sys

# ==================== 配置 ====================
HC06_PORT = "COM16"     # HC-06 从机
HC05_PORT = "COM53"     # HC-05 主机
BAUDRATE = 115200
TIMEOUT = 2.0           # 串口读取超时(秒)
CMD_DELAY = 0.5         # 命令间延时(秒)

# ==================== 工具函数 ====================

def open_port(port_name, label):
    """打开串口"""
    try:
        ser = serial.Serial(port_name, BAUDRATE, timeout=TIMEOUT)
        print(f"[OK] {label} ({port_name}) 已打开 @ {BAUDRATE}")
        return ser
    except Exception as e:
        print(f"[FAIL] 无法打开 {label} ({port_name}): {e}")
        return None


def send_at(ser, cmd, wait=CMD_DELAY):
    """发送 AT 命令并读取响应"""
    ser.reset_input_buffer()
    full_cmd = cmd + "\r\n"
    ser.write(full_cmd.encode())
    time.sleep(wait)
    lines = []
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode(errors='replace').strip()
            if line:
                lines.append(line)
        except:
            break
    return lines


def close_port(ser, label):
    """关闭串口"""
    try:
        ser.close()
        print(f"[OK] {label} 串口已关闭")
    except:
        pass


def parse_mac(response_lines):
    """从 AT+ADDR? 的响应中解析 MAC 地址 (格式: 1234:56:789ABC 或 12:34:56:78:9A:BC)"""
    for line in response_lines:
        # HC-06: +ADDR:1234:56:789ABC 或直接返回地址
        for pat in [r'\+ADDR[:=]\s*([0-9A-Fa-f:]+)', r'^([0-9A-Fa-f]{1,4}:[0-9A-Fa-f]{1,4}:[0-9A-Fa-f]{1,8})$']:
            m = re.search(pat, line, re.IGNORECASE)
            if m:
                return m.group(1).upper()
    return None


def mac_to_bind_format(mac):
    """
    将 MAC 地址转换为 AT+BIND 所需的逗号分隔格式
    输入: "1234:56:789ABC" 或 "12:34:56:78:9A:BC"
    输出: "1234,56,789ABC"
    """
    parts = mac.replace(":", ",").split(",")
    # 清理前导零等
    return ",".join(parts)


# ==================== Step 1: 读取 HC-06 地址 ====================

def step1_read_hc06_addr():
    print("\n" + "=" * 60)
    print("  Step 1: 读取 HC-06 (COM16) 蓝牙地址")
    print("=" * 60)

    ser = open_port(HC06_PORT, "HC-06")
    if not ser:
        return None

    # 先发几次 AT 确保模块在 AT 模式
    for i in range(3):
        resp = send_at(ser, "AT")
        print(f"  AT test #{i+1}: {resp}")

    # 查询地址
    print("\n  查询地址...")
    resp = send_at(ser, "AT+ADDR?", wait=1.0)
    print(f"  AT+ADDR? 响应: {resp}")
    
    mac = parse_mac(resp)
    if mac:
        print(f"\n  >>> HC-06 MAC 地址: {mac}")
    else:
        # 有些 HC-06 的响应格式不同，尝试更多命令
        for alt_cmd in ["AT+ADDR", "AT+ADDRESS?", "AT+ADDRESS"]:
            resp = send_at(ser, alt_cmd, wait=1.0)
            print(f"  {alt_cmd} 响应: {resp}")
            mac = parse_mac(resp)
            if mac:
                print(f"\n  >>> HC-06 MAC 地址: {mac}")
                break

    close_port(ser, "HC-06")
    return mac


# ==================== Step 2: 配置 HC-05 主机并绑定 ====================

def step2_config_hc05(target_mac):
    print("\n" + "=" * 60)
    print("  Step 2: 配置 HC-05 (COM53) 主机模式并绑定")
    print("=" * 60)

    ser = open_port(HC05_PORT, "HC-05")
    if not ser:
        return False

    # 测试 AT 连接
    print("\n  测试连接...")
    resp = send_at(ser, "AT")
    print(f"  AT 响应: {resp}")

    # 检查是否有 OK
    has_ok = any("OK" in r for r in resp)
    if not has_ok:
        # 可能已经进入透传模式，多试几次
        for i in range(3):
            resp = send_at(ser, "AT")
            print(f"  AT 重试 #{i+1}: {resp}")
            if any("OK" in r for r in resp):
                has_ok = True
                break

    if not has_ok:
        print("\n[WARN] HC-05 可能不在 AT 模式！")
        print("  请确保: HC-05 上电前按住按键/EN引脚拉高，LED慢闪表示AT模式")
        close_port(ser, "HC-05")
        return False

    # 查询当前角色
    print("\n  查询当前角色...")
    resp = send_at(ser, "AT+ROLE?")
    print(f"  AT+ROLE? 响应: {resp}")

    # 设置为主机模式 (ROLE=1)
    print("\n  设置为主机模式 (ROLE=1)...")
    resp = send_at(ser, "AT+ROLE=1")
    print(f"  AT+ROLE=1 响应: {resp}")

    # 设置连接模式为固定地址 (CMODE=0)
    print("\n  设置固定地址配对模式 (CMODE=0)...")
    resp = send_at(ser, "AT+CMODE=0")
    print(f"  AT+CMODE=0 响应: {resp}")

    # 绑定目标地址
    bind_addr = mac_to_bind_format(target_mac)
    print(f"\n  绑定 HC-06 地址: {target_mac}")
    print(f"  BIND 格式: {bind_addr}")
    
    resp = send_at(ser, f"AT+BIND={bind_addr}")
    print(f"  AT+BIND={bind_addr} 响应: {resp}")

    # 查询绑定确认
    print("\n  验证绑定...")
    resp = send_at(ser, "AT+BIND?")
    print(f"  AT+BIND? 响应: {resp}")

    # 初始化 SPP 配置文件 (某些固件需要)
    print("\n  初始化...")
    resp = send_at(ser, "AT+INIT")
    print(f"  AT+INIT 响应: {resp}")

    # 保存并重启 (或提示用户断电重启)
    print("\n  重启模块使设置生效...")
    resp = send_at(ser, "AT+RESET")
    print(f"  AT+RESET 响应: {resp}")

    close_port(ser, "HC-05")
    print("\n[INFO] 配对配置完成！")
    print("  HC-05 重启后将自动搜索并连接 HC-06")
    print("  LED 快闪(搜索) → 快闪变慢闪(已连接)")
    print("  连接成功后两模块可透传数据")
    return True


# ==================== 主流程 ====================

def main():
    print("=" * 60)
    print("  HC-05 ↔ HC-06 蓝牙配对工具")
    print(f"  HC-06 从机: {HC06_PORT} @ {BAUDRATE}")
    print(f"  HC-05 主机: {HC05_PORT} @ {BAUDRATE}")
    print("=" * 60)
    print("\n[INFO] 请确保:")
    print("  1. 两模块均已上电")
    print("  2. HC-05 已进入 AT 模式 (LED 慢闪，约 2Hz)")
    print("  3. HC-06 未与其他设备连接")

    # Step 1: 读取 HC-06 地址
    mac = step1_read_hc06_addr()
    if not mac:
        print("\n[FAIL] 无法读取 HC-06 的 MAC 地址！")
        print("  请检查:")
        print("  - COM16 是否正确")
        print("  - HC-06 是否上电且未连接其他设备")
        print("  - 波特率是否为 115200")
        sys.exit(1)

    # Step 2: 配置 HC-05
    success = step2_config_hc05(mac)
    if success:
        print("\n" + "=" * 60)
        print("  ✅ 配对脚本执行完毕！")
        print(f"  HC-05 (COM53) → HC-06 (COM16, {mac})")
        print("  HC-05 重启后将自动连接 HC-06")
        print("=" * 60)
    else:
        print("\n[FAIL] 配对失败，请检查 HC-05 是否在 AT 模式")
        sys.exit(1)


if __name__ == "__main__":
    main()