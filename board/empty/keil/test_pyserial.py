import serial
import serial.tools.list_ports

print("pyserial version:", serial.__version__)

ports = serial.tools.list_ports.comports()
print("COM ports found:", len(ports))
for p in ports:
    print(f"  {p.device} | {p.manufacturer} | {p.description}")