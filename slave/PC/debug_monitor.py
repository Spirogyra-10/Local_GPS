"""
debug_monitor.py
Arduino - PC 間のシリアル通信デバッグ用ターミナルモニタ
- COM ポートの自動検出・ポーリング接続
- リアルタイムログ表示
- キーボードからのコマンド送信 (デバッグ切替 'd', 設定コマンド '#1', '#SET9600', 'AT' 等)
"""

import argparse
import sys
import threading
import time
import serial
import serial.tools.list_ports


def list_ports():
    return list(serial.tools.list_ports.comports())


def auto_detect_port():
    print("[Monitor] Scanning for connected serial devices...")
    while True:
        ports = list_ports()
        if ports:
            print(f"[Monitor] Detected {len(ports)} port(s):")
            for idx, p in enumerate(ports):
                print(f"  [{idx + 1}] {p.device} - {p.description}")
            return ports[0].device
        print("[Monitor] No devices found. Please connect Arduino Uno via USB... (retrying in 2s)")
        time.sleep(2.0)


def reader_thread(ser, stop_event):
    while not stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="replace")
                if line:
                    sys.stdout.write(line)
                    sys.stdout.flush()
            else:
                time.sleep(0.01)
        except Exception as e:
            if not stop_event.is_set():
                print(f"\n[Monitor Error] Connection lost: {e}")
            break


def main():
    parser = argparse.ArgumentParser(description="Local GPS Arduino Serial Debug Monitor")
    parser.add_argument("--port", type=str, default=None, help="COM port (e.g. COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    args = parser.parse_args()

    port = args.port
    if not port:
        port = auto_detect_port()

    print(f"\n[Monitor] Connecting to {port} at {args.baud} bps...")
    try:
        ser = serial.Serial(port, args.baud, timeout=0.5)
        ser.reset_input_buffer()
        print(f"[Monitor] Connected successfully to {port}!")
        print("--------------------------------------------------")
        print("Tips:")
        print("  - If running slave_controller: Send 'd' to toggle verbose debug logs.")
        print("  - If running slave_config_tool: Send '#1', '#2', '#3', '#SET9600', 'AT' etc.")
        print("  - Press Ctrl+C to exit.")
        print("--------------------------------------------------\n")
    except Exception as e:
        print(f"[Error] Failed to open {port}: {e}")
        return

    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    try:
        while True:
            cmd = input()
            if not ser.is_open:
                break
            # CR+LF を付加して送信
            ser.write((cmd + "\r\n").encode("utf-8"))
    except KeyboardInterrupt:
        print("\n[Monitor] Closing monitor...")
    finally:
        stop_event.set()
        if ser.is_open:
            ser.close()
        print("[Monitor] Done.")


if __name__ == "__main__":
    main()

