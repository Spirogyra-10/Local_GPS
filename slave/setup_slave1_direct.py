import time
import serial

def send_and_wait(ser, cmd, delay=0.3):
    print(f"\n-> TX: {cmd}")
    ser.write((cmd + "\r\n").encode("utf-8"))
    time.sleep(delay)
    res = ser.read_all().decode("utf-8", errors="replace").strip()
    print(f"<- RX: {repr(res)}")
    return res

print("==========================================")
print(" Configuring SLAVE 1 directly at 115200   ")
print("==========================================")

ser = serial.Serial("COM5", 115200, timeout=0.5)
time.sleep(1.0)
ser.reset_input_buffer()

# 1. 導通確認
send_and_wait(ser, "AT")

# 2. Anchor モード設定
send_and_wait(ser, "AT+MODE=1")

# 3. 共通 Network ID 設定
send_and_wait(ser, "AT+NETWORKID=LOCALGPS")

# 4. アドレス設定 (SLAVE1)
send_and_wait(ser, "AT+ADDRESS=SLAVE1")

# 5. 設定確認
send_and_wait(ser, "AT+MODE?")
send_and_wait(ser, "AT+NETWORKID?")
send_and_wait(ser, "AT+ADDRESS?")

# 6. ボーレートを 9600 bps に変更
print("\n>>> Setting Baudrate to 9600 bps... <<<")
send_and_wait(ser, "AT+IPR=9600")

ser.close()
time.sleep(0.5)

# 7. 9600 bps で再接続して確認
print("\n==========================================")
print(" Verifying SLAVE 1 at 9600 bps            ")
print("==========================================")
ser96 = serial.Serial("COM5", 9600, timeout=0.5)
time.sleep(0.5)
ser96.reset_input_buffer()

res = send_and_wait(ser96, "AT")
if "+OK" in res or "OK" in res:
    print("\n==================================================")
    print(" >>> SUCCESS: SLAVE 1 IS FULLY CONFIGURED! <<<    ")
    print("==================================================")
else:
    print("\n[!] Check 9600 response:", res)

ser96.close()
