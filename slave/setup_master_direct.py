import time
import serial
import serial.tools.list_ports

def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "arduino" in desc or "2341:0043" in hwid or "ch340" in desc or "usb" in desc:
            return p.device
    if ports:
        return ports[0].device
    return None

def send_and_wait(ser, cmd, delay=0.3):
    print(f"\n-> TX: {cmd}")
    ser.write((cmd + "\r\n").encode("utf-8"))
    time.sleep(delay)
    res = ser.read_all().decode("utf-8", errors="replace").strip()
    print(f"<- RX: {repr(res)}")
    return res

print("==========================================")
print(" Configuring MASTER (TAG1)                ")
print("==========================================")

port = find_port()
if not port:
    print("[Error] COM port not detected.")
    exit(1)

print(f"[Detected] Using port: {port}")

# まず初期速度 115200 bps で接続試行
ser = None
for baud in [115200, 9600]:
    try:
        s = serial.Serial(port, baud, timeout=0.4)
        time.sleep(0.5)
        s.reset_input_buffer()
        s.write(b"AT\r\n")
        time.sleep(0.3)
        res = s.read_all().decode("utf-8", errors="replace").strip()
        if "+OK" in res or "OK" in res:
            print(f"Connected to MASTER at {baud} bps!")
            ser = s
            break
        s.close()
    except Exception:
        pass

if not ser:
    print("[Error] Failed to communicate with MASTER module.")
    exit(1)

# 1. Tag モード設定 (AT+MODE=0)
send_and_wait(ser, "AT+MODE=0")

# 2. 共通 Network ID 設定 (AT+NETWORKID=LOCALGPS)
send_and_wait(ser, "AT+NETWORKID=LOCALGPS")

# 3. アドレス設定 (AT+ADDRESS=TAG1)
send_and_wait(ser, "AT+ADDRESS=TAG1")

# 4. 設定確認
send_and_wait(ser, "AT+MODE?")
send_and_wait(ser, "AT+NETWORKID?")
send_and_wait(ser, "AT+ADDRESS?")

print("\n==================================================")
print(" >>> SUCCESS: MASTER (TAG1) FULLY CONFIGURED! <<< ")
print("==================================================")

ser.close()
