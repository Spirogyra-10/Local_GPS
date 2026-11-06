"""
upload.py
Arduino IDE なしで Arduino Uno へのスケッチコンパイル＆書き込みを行うユーティリティ
PlatformIO Core (pio) をバックエンドとして使用します。
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import serial.tools.list_ports


SKETCHES = {
    "config": os.path.join("slave_config_tool", "slave_config_tool.ino"),
    "controller": os.path.join("slave_controller", "slave_controller.ino"),
    "diag": os.path.join("slave_diagnostics", "slave_diagnostics.ino"),
}


def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "arduino" in desc or "2341:0043" in hwid or "ch340" in desc:
            return p.device
    if ports:
        return ports[0].device
    return None


def upload_sketch(sketch_path, port):
    # 一時的な PlatformIO プロジェクトを作成
    temp_dir = tempfile.mkdtemp(prefix="pio_build_")
    src_dir = os.path.join(temp_dir, "src")
    os.makedirs(src_dir, exist_ok=True)

    # .ino ファイルをコピー (拡張子 .cpp または .ino のまま)
    dest_file = os.path.join(src_dir, os.path.basename(sketch_path))
    shutil.copy2(sketch_path, dest_file)

    # platformio.ini を作成
    ini_content = f"""[env:uno]
platform = atmelavr
board = uno
framework = arduino
upload_port = {port}
upload_speed = 115200
"""
    with open(os.path.join(temp_dir, "platformio.ini"), "w") as f:
        f.write(ini_content)

    print(f"Building and uploading in: {temp_dir}")
    try:
        cmd = ["pio", "run", "-t", "upload", "-d", temp_dir]
        res = subprocess.run(cmd)
        return res.returncode == 0
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def main():
    parser = argparse.ArgumentParser(description="Upload Arduino Sketch without Arduino IDE")
    parser.add_argument(
        "target",
        choices=["config", "controller", "diag"],
        help="Target sketch to upload: 'config', 'controller', or 'diag'",
    )
    parser.add_argument("--port", type=str, default=None, help="COM port (e.g. COM5)")
    args = parser.parse_args()

    port = args.port or find_arduino_port()
    if not port:
        print("[Error] No Arduino Uno detected! Please connect it via USB.")
        sys.exit(1)

    sketch_path = SKETCHES[args.target]
    if not os.path.exists(sketch_path):
        print(f"[Error] Sketch file not found: {sketch_path}")
        sys.exit(1)

    print("==================================================")
    print(f"  Uploading Target: {args.target}")
    print(f"  Sketch File:     {sketch_path}")
    print(f"  Target Port:     {port}")
    print("==================================================")

    success = upload_sketch(sketch_path, port)

    if success:
        print("\n==================================================")
        print(f" SUCCESS: {args.target} successfully uploaded to {port}!")
        print("==================================================")
    else:
        print(f"\n[Failed] Upload failed.")
        sys.exit(1)


if __name__ == "__main__":
    main()
