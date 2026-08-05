#!/usr/bin/env python3
"""
Verify that Appli/App/link_proto.h and pi/nexus_proto.py describe the same
bytes, field by field.

A protocol mismatch between the two sides is silent and miserable to debug:
every value the Pi reads is subtly wrong, and it looks like broken sensors.
This compiles a tiny C program that prints the real offsetof() for every
field, then checks each against the Python struct format.

    python tools/check_proto.py
"""

import re
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "pi"))

import nexus_proto as P  # noqa: E402

# (C field name, python format chunk, count) in wire order.
FIELDS = [
    ("sync",             "H", 1),
    ("msg_id",           "B", 1),
    ("version",          "B", 1),
    ("seq",              "I", 1),
    ("timestamp_us",     "I", 1),
    # ---- policy block ----
    ("pelvis_z",         "f", 1),
    ("quat",             "f", 4),
    ("gyro",             "f", 3),
    ("vel_hdg",          "f", 3),
    ("joint_pos",        "f", P.NUM_JOINTS),
    ("joint_vel",        "f", P.NUM_JOINTS),
    ("spring_angle",     "f", P.NUM_ENCODERS),
    ("ref_angle",        "f", P.NUM_JOINTS),
    ("contact",          "f", P.NUM_CONTACTS),
    ("foot_z",           "f", 2),
    ("phase",            "f", 1),
    # ---- raw IMU ----
    ("imu_quat",         "f", 4),
    ("imu_accel",        "f", 3),
    ("imu_gyro",         "f", 3),
    ("imu_seq",          "I", 1),
    # ---- actuator diagnostics ----
    ("act_torque",       "f", P.NUM_JOINTS),
    ("act_error",        "I", P.NUM_JOINTS),
    # ---- estimator internals ----
    ("fused_pos",        "f", 3),
    ("fused_vel",        "f", 3),
    ("fused_gyro_bias",  "f", 3),
    ("fused_accel_bias", "f", 3),
    ("contact_ticks",    "H", 2),
    ("act_state",        "B", P.NUM_JOINTS),
    ("act_flags",        "B", P.NUM_JOINTS),
    ("enc_valid",        "B", 1),
    ("contacts",         "B", 1),
    ("fused_valid",      "B", 1),
    ("health",           "B", 1),
    ("crc",              "H", 1),
]


def c_offsets():
    """Compile a probe that prints offsetof() for every field."""
    lines = ['#include <stdio.h>', '#include <stddef.h>', '#include "link_proto.h"',
             'int main(void){']
    for name, _, _ in FIELDS:
        lines.append(f'  printf("{name} %zu\\n", offsetof(nexus_state_t, {name}));')
    lines.append('  printf("__size__ %zu\\n", sizeof(nexus_state_t));')
    lines.append('  printf("__cmd__ %zu\\n", sizeof(nexus_cmd_t));')
    lines.append('  return 0;}')

    with tempfile.TemporaryDirectory() as td:
        src = Path(td) / "probe.c"
        exe = Path(td) / "probe.exe"
        src.write_text("\n".join(lines))
        subprocess.run(
            ["gcc", "-I", str(ROOT / "Appli" / "App"), str(src), "-o", str(exe)],
            check=True, capture_output=True,
        )
        out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout

    return {k: int(v) for k, v in (l.split() for l in out.strip().splitlines())}


def py_offsets():
    """Walk the Python format string accumulating offsets."""
    off, res = 0, {}
    for name, fmt, count in FIELDS:
        res[name] = off
        off += struct.calcsize("<" + fmt) * count
    res["__size__"] = off
    return res


def main():
    c = c_offsets()
    p = py_offsets()

    print(f"{'field':<20}{'C':>8}{'Python':>10}   ")
    print("-" * 42)
    bad = 0
    for name, _, _ in FIELDS:
        ok = c[name] == p[name]
        bad += not ok
        print(f"{name:<20}{c[name]:>8}{p[name]:>10}   {'ok' if ok else 'MISMATCH'}")

    print("-" * 42)
    for key, pyval in (("__size__", p["__size__"]),):
        ok = c[key] == pyval
        bad += not ok
        print(f"{'sizeof(state)':<20}{c[key]:>8}{pyval:>10}   {'ok' if ok else 'MISMATCH'}")

    ok = c["__cmd__"] == P.COMMAND_SIZE
    bad += not ok
    print(f"{'sizeof(cmd)':<20}{c['__cmd__']:>8}{P.COMMAND_SIZE:>10}   {'ok' if ok else 'MISMATCH'}")

    # Every 4-byte field must be 4-aligned or the M7 pays for byte-wise access
    # and numpy cannot view the buffer in place.
    print()
    mis = [n for n, f, _ in FIELDS if f in "fI" and c[n] % 4]
    if mis:
        bad += 1
        print("MISALIGNED 4-byte fields:", ", ".join(mis))
    else:
        print("all 4-byte fields are 4-byte aligned")

    # The policy block must be one contiguous run of float32 at a known
    # offset, or the zero-copy numpy slice on the Pi reads the wrong bytes.
    if c["pelvis_z"] != P.POLICY_OFFSET:
        bad += 1
        print("POLICY BLOCK starts at", c["pelvis_z"], "expected", P.POLICY_OFFSET)
    elif (c["imu_quat"] - c["pelvis_z"]) != P.POLICY_COUNT * 4:
        bad += 1
        print("POLICY BLOCK not contiguous:",
              c["imu_quat"] - c["pelvis_z"], "vs", P.POLICY_COUNT * 4)
    else:
        print(f"policy block: {P.POLICY_COUNT} float32 at offset "
              f"{P.POLICY_OFFSET}, contiguous")

    print("\n" + ("PROTOCOL MISMATCH" if bad else "C and Python agree"))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
