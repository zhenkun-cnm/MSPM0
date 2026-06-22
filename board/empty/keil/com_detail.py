import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
print(f"找到 {len(ports)} 个串口：")
print("=" * 60)

for i, p in enumerate(ports, 1):
    import re
    hwid = p.hwid or ""
    vid_match = re.search(r'VID_([0-9A-Fa-f]{4})', hwid)
    pid_match = re.search(r'PID_([0-9A-Fa-f]{4})', hwid)
    vid = vid_match.group(1) if vid_match else ""
    pid = pid_match.group(1) if pid_match else ""

    print(f"  [{i}] {p.device}")
    print(f"      制造商: {p.manufacturer or '未知'}")
    print(f"      描述: {p.description or p.device}")
    if vid and pid:
        print(f"      VID:PID = {vid}:{pid}")
    if p.serial_number:
        print(f"      序列号: {p.serial_number}")
    if p.location:
        print(f"      位置: {p.location}")
    print()

print("=" * 60)