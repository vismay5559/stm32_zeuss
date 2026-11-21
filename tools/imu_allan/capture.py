#!/usr/bin/env python3
"""Record the IMU_LOG stream to a file, with a status line every 10 s.

    python3 tools/imu_allan/capture.py --minutes 60 --out imu_still.bin

Flash NEXUS_MODE_IMU_LOG first. The file is the raw serial bytes, so nothing
is lost to a decoder bug: allan.py decodes it afterwards.

The robot must not move for the whole run. A bump shows up as a step in the
status line's gyro; note the time, and allan.py can cut it out (--skip/--stop).
"""
import argparse
import glob
import os
import sys
import time

import numpy as np
import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imu_frames import FRAME_HZ, decode  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None,
                    help="default: the ST-LINK console, by id")
    ap.add_argument("--minutes", type=float, default=60.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    port = a.port
    if port is None:
        hits = glob.glob("/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3*-if01")
        if not hits:
            sys.exit("no ST-LINK console found; pass --port")
        port = hits[0]

    s = serial.Serial(port, 115200, timeout=0.2, exclusive=True)
    end = time.time() + a.minutes * 60.0
    total = bytearray()
    next_status = time.time() + 10.0
    seen = 0
    print(f"recording {port} for {a.minutes:g} min into {a.out} - keep the robot still")

    with open(a.out, "wb") as f:
        while time.time() < end:
            chunk = s.read(8192)
            if chunk:
                f.write(chunk)
                total += chunk
            if time.time() >= next_status:
                next_status += 10.0
                fast, slow, bad, _ = decode(bytes(total))
                if len(fast) == 0:
                    print("  no frames yet (is IMU_LOG flashed? did the IMU start?)")
                    continue
                new = fast[seen:]
                seen = len(fast)
                span = fast[-1, 0] - fast[0, 0] + 1
                lost = span - len(fast)
                dropped = int(slow[-1, 3]) if len(slow) else -1
                left = (end - time.time()) / 60.0
                if len(new):
                    acc = new[:, 1:4].mean(axis=0)
                    gyr = new[:, 4:7].mean(axis=0)
                    print(f"  {len(fast) / FRAME_HZ / 60:5.1f} min  lost={int(lost)}  "
                          f"badsum={bad}  console_dropped={dropped}  "
                          f"a=[{acc[0]:+.3f} {acc[1]:+.3f} {acc[2]:+.3f}]  "
                          f"w=[{gyr[0]:+.5f} {gyr[1]:+.5f} {gyr[2]:+.5f}]  "
                          f"{left:.1f} min left", flush=True)
                # Keep memory bounded on long runs: only the tail is needed
                # for the next status line.
                if len(total) > 4_000_000:
                    total = total[-200_000:]
                    seen = 0
                    fast, _, _, _ = decode(bytes(total))
                    seen = len(fast)
    print(f"done: {os.path.getsize(a.out)} bytes in {a.out}")


if __name__ == "__main__":
    main()
