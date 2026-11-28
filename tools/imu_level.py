#!/usr/bin/env python3
"""
Correct the board's IMU table from a `zeus imu-level` measurement.

    python3 tools/imu_level.py imu_level_20261128_175125.json
    python3 tools/imu_level.py imu_level_*.json --dry-run      # say, change nothing

`zeus imu-level` (zeus_8dof_deploy) records the gravity the board reports
while the torso is held level against an outside reference - a level on the
torso reading 0. A correct table reads that as pitch 0, roll 0; whatever it
reads instead is the table's error. This turns the table by exactly the
rotation that brings that gravity vertical, the smallest one, so the axes it
was built on (chip X right, Y forward, Z up, measured 2026-11-28) do not move.

It rewrites, in place:
  Appli/App/robot_config.c            g_imu_mount_R and g_imu_mount_q, and a
                                      dated note of what was measured
  tools/hosttest/test_robot_config.c  the "straight" vector (that measurement,
                                      in chip axes) and what the forward tilt
                                      must now read

Then: bash tools/hosttest/run.sh, build, flash - and relaunch the robot
software with the torso held at 0 on the same level, so the ankles zero there.

Under 0.5 deg of error it does nothing: that is inside what holding a robot
still against a level can promise, and a flash for less is noise chasing.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / "Appli" / "App" / "robot_config.c"
TEST = ROOT / "tools" / "hosttest" / "test_robot_config.c"
MIN_CORRECTION_DEG = 0.5

_R_RE = re.compile(r"const float g_imu_mount_R\[9\] = \{(.*?)\};", re.S)
_Q_RE = re.compile(r"const float g_imu_mount_q\[4\] = \{(.*?)\};", re.S)
_FLOAT = re.compile(r"[-+]?\d*\.\d+(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?")


def floats(text: str):
    return [float(v) for v in _FLOAT.findall(text.replace("f", ""))]


def tilt(g):
    g = np.asarray(g, dtype=np.float64)
    g = g / np.linalg.norm(g)
    return (math.degrees(math.atan2(-g[0], math.hypot(g[1], g[2]))),
            math.degrees(math.atan2(g[1], g[2])))


def rotation_onto_up(g):
    """The smallest rotation taking direction g onto +Z (Rodrigues)."""
    g = np.asarray(g, dtype=np.float64)
    g = g / np.linalg.norm(g)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(g, z)
    s, c = float(np.linalg.norm(v)), float(g @ z)
    if s < 1e-12:
        return np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * ((1 - c) / s ** 2)


def quat_of(m):
    """(w, x, y, z) of a rotation matrix (trace form; fine away from 180 deg)."""
    w = math.sqrt(max(0.0, 1.0 + float(np.trace(m)))) / 2.0
    return [w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)]


def corrected(r_old: np.ndarray, gravity_body) -> np.ndarray:
    """The table that reads gravity_body (as reported through r_old) as level."""
    return rotation_onto_up(gravity_body) @ r_old


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("measurement", help="imu_level_*.json from `zeus imu-level`")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)

    m = json.loads(Path(a.measurement).read_text())
    if not m.get("still", False):
        print("that measurement was not taken still: " + "; ".join(m.get("why_not", [])))
        return 2
    g_body = np.asarray(m["gravity_body"], dtype=np.float64)
    pitch, roll = tilt(g_body)
    print(f"{Path(a.measurement).name}: the level torso read pitch {pitch:+.2f}, roll {roll:+.2f} deg")
    if max(abs(pitch), abs(roll)) < MIN_CORRECTION_DEG:
        print(f"under {MIN_CORRECTION_DEG} deg - the table is right, nothing changed")
        return 0

    cfg = CONFIG.read_text()
    r_old = np.array(floats(_R_RE.search(cfg).group(1))).reshape(3, 3)
    r_new = corrected(r_old, g_body)
    q_new = quat_of(r_new.T)
    chip_up = r_old.T @ g_body                       # the measurement in chip axes
    assert max(abs(v) for v in tilt(r_new @ chip_up)) < 1e-3   # 7-digit table: ~1e-5 deg

    test = TEST.read_text()
    fwd = re.search(r"const float g_fwd\[3\] = \{([^}]*)\};", test)
    fwd_pitch, fwd_roll = tilt(r_new @ np.array(floats(fwd.group(1))))
    print(f"correction {math.degrees(math.acos(min(1.0, (np.trace(r_new @ r_old.T) - 1) / 2))):.2f} deg; "
          f"the forward-tilt test vector now reads pitch {fwd_pitch:+.2f}, roll {fwd_roll:+.2f}")
    if a.dry_run:
        return 0

    f = lambda v: "%+.7ff" % v
    rows = ",\n".join("    " + ", ".join(f(v) for v in r_new[i]) for i in range(3))
    note = (f" * LEVELLED {m.get('time', '?')} against a level on the torso at 0 (zeus imu-level,\n"
            f" * tools/imu_level.py): the old table read that as pitch {pitch:+.2f}, roll {roll:+.2f}\n"
            f" * deg; turned by the smallest rotation that reads it level. Chip gravity\n"
            f" * then: ({chip_up[0]:.4f}, {chip_up[1]:.4f}, {chip_up[2]:.4f}) m/s^2.\n */\n")
    cfg = _R_RE.sub("const float g_imu_mount_R[9] = {\n" + rows + ",\n};", cfg, count=1)
    cfg = _Q_RE.sub("const float g_imu_mount_q[4] = { " + ", ".join(f(v) for v in q_new) + " };", cfg, count=1)
    i = cfg.index("const float g_imu_mount_R[9]")
    end = cfg.rfind(" */\n", 0, i)
    cfg = cfg[:end] + " *\n" + note + cfg[end + len(" */\n"):]
    CONFIG.write_text(cfg)

    test = re.sub(r"const float g_chip\[3\] = \{[^}]*\};[^\n]*",
                  "const float g_chip[3] = { %.4ff, %.4ff, %.4ff };   /* level, %s */"
                  % (*chip_up, m.get("time", "?")), test, count=1)
    # The forward-tilt check: written as "roll" on the first levelling, then as
    # "roll - (x)" - either form, so the script can run again on its own output.
    test, n = re.subn(r"CHECK\(fabsf\(roll(?: - \([-+]?\d+\.\d+f\))?\) < 0\.1f && "
                      r"fabsf\(pitch - \(?[-+]?\d+\.\d+f\)?\) < 0\.2f,",
                      "CHECK(fabsf(roll - (%+.2ff)) < 0.1f && fabsf(pitch - (%+.2ff)) < 0.2f,"
                      % (fwd_roll, fwd_pitch), test, count=1)
    if n != 1:
        print(f"could not find the forward-tilt check in {TEST.name} - table changed, test NOT updated")
        CONFIG.write_text(cfg)
        return 1
    TEST.write_text(test)
    print(f"rewrote {CONFIG.relative_to(ROOT)} and {TEST.relative_to(ROOT)}: "
          "now bash tools/hosttest/run.sh, build, flash")
    return 0


if __name__ == "__main__":
    sys.exit(main())
