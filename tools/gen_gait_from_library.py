#!/usr/bin/env python3
"""
Write Appli/App/gait_ref.c/.h from a policy's gait library.

    python3 tools/gen_gait_from_library.py --speed 0.30 --run z1_softland_tol05

The library is VENDORED at tools/model/gait_library.npz, for the same reason
the URDF is: this repo is private, the generator then needs nothing external,
and a regeneration here and one in CI read the same bytes.

WHY THIS EXISTS, rather than the workspace's own exporter.

`zeus_control_interface/scripts/export_policy.py` writes these files on the
TRAINING machine, as one step of freezing a run into a bundle. It is the right
place for it and this script is not trying to replace it. But its C template
has SLOTS fixed at five joints a leg including ankle roll, so for the 8-DOF
robot it emits a table this board cannot play - which is exactly why the
z1_softland_tol05 bundle shipped with no gait_ref.c at all, and said so.

Everything the C tables need is in the library npz. Nothing here needs SB3,
MuJoCo or a trained checkpoint, so it runs anywhere. The emitted code is
deliberately identical in shape to what export_policy.py produces, so that
when its SLOTS is fixed for four joints the two agree and this file can go.

ONE DELIBERATE DIFFERENCE. The exporter's header hardcodes

    #define GAIT_COL_ANKLE_ROLL  4

whatever SLOTS contains. With four columns that names an index past the end of
every row, and a define that lies is worse than one that is missing. It is
emitted only when there is a column for it.

Checks, all of them refusing to write rather than warning:

  - the library's joint_order is the firmware's joint order, name for name.
    The tables are indexed by position, so a library that lists its joints
    differently would silently transpose the robot's legs.
  - the requested speed is a library ROW, not between two. A row is a solved
    periodic orbit; an interpolation between two is not a gait.
  - the library is half-stride mirror symmetric to 1e-9, negating the roll
    joints. The board plays ONE leg's columns for both legs, the left half a
    stride behind with roll flipped, so if this did not hold the left leg
    would walk a gait nothing ever trained on. This is the same assertion
    export_policy.py makes, for the same reason.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, "Appli", "App")

# The firmware's joint order, NEXUS_J_* in link_proto.h. Checked against the
# library rather than assumed.
JOINT_ORDER = [f"{s}_{j}" for s in ("left", "right")
               for j in ("hip_pitch", "hip_roll", "knee_pitch", "ankle_pitch")]

# gait_ref columns, right leg only. Same order export_policy.py uses, minus
# the ankle roll this robot does not have.
SLOTS = ["hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch"]

COL_DEFINE = {"hip_roll": "GAIT_COL_HIP_ROLL", "hip_pitch": "GAIT_COL_HIP_PITCH",
              "knee_pitch": "GAIT_COL_KNEE", "ankle_pitch": "GAIT_COL_ANKLE",
              "ankle_roll": "GAIT_COL_ANKLE_ROLL"}


def sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def mirror_matrix(names) -> np.ndarray:
    """Left<->right, negating anything that rolls. zeus_model.mirror_matrix."""
    n = len(names)
    S = np.zeros((n, n))
    for i, nm in enumerate(names):
        side, joint = nm.split("_", 1)
        j = names.index(("right_" if side == "left" else "left_") + joint)
        S[i, j] = -1.0 if joint.endswith("roll") else 1.0
    return S


def table(name: str, arr) -> str:
    rows = ",\n".join("    { " + ", ".join(f"{v:+.7f}f" for v in r) + " }"
                      for r in arr)
    return f"const float {name}[GAIT_SAMPLES][GAIT_JOINTS] = {{\n{rows}\n}};\n"


def build(lib_path: str, speed: float, run: str):
    L = np.load(lib_path, allow_pickle=True)
    names = [str(x) for x in L["joint_order"]]

    if names != JOINT_ORDER:
        raise SystemExit(
            "the library's joint order is not the firmware's:\n"
            f"  library  {names}\n  firmware {JOINT_ORDER}\n"
            "The tables are indexed by position, so writing them anyway would "
            "transpose the robot's joints without a word.")

    vel = np.asarray(L["velocities"], dtype=float)
    hits = np.flatnonzero(np.abs(vel - speed) < 1e-9)
    if hits.size != 1:
        raise SystemExit(
            f"{speed} is not a library row. Rows: {np.round(vel, 3).tolist()}\n"
            "A row is a solved periodic orbit; a blend of two is not a gait.")
    row = int(hits[0])

    q = np.asarray(L["q_joints"][row], dtype=float)
    dq = np.asarray(L["dq_joints"][row], dtype=float)
    tau = np.asarray(L["tau_joints"][row], dtype=float)
    T = float(np.asarray(L["T"], dtype=float)[row])

    # The board plays one leg's columns for both legs. If this did not hold,
    # the left leg would get a gait nothing trained on.
    S, half = mirror_matrix(names), q.shape[0] // 2
    for nm, arr in (("q", q), ("dq", dq), ("tau", tau)):
        err = float(np.abs(np.roll(arr, -half, axis=0) @ S.T - arr).max())
        if err >= 1e-9:
            raise SystemExit(
                f"the library is not half-stride mirror symmetric in {nm}: "
                f"worst {err:.3e}, needs < 1e-9. The firmware cannot play one "
                f"leg's columns for both.")

    idx = [names.index("right_" + s) for s in SLOTS]
    n = q.shape[0]
    tp = 2.0 * np.pi

    cols = "\n".join(f"#define {COL_DEFINE[s]:<20} {i}"
                     for i, s in enumerate(SLOTS))

    h = f"""#ifndef GAIT_REF_H
#define GAIT_REF_H

#include <stdint.h>

/*
 * Reference gait for the right leg. GENERATED by
 * tools/gen_gait_from_library.py from
 *   {os.path.basename(lib_path)}
 *   sha256 {sha256(lib_path)}
 * row v = {speed:.2f} m/s. DO NOT EDIT - regenerate.
 *
 * This is the stride policy {run} was TRAINED on. Replacing it with
 * any other table invalidates the policy, and rl_policy_node refuses to enable
 * if ref_angle differs from the copy it carries.
 *
 * Rows are OUTPUT-side values, in the URDF sign convention. Scale them to each
 * drive's own turns (the ankle encoder sits on the motor) where the command is
 * sent, not here.
 *
 *   g_gait_turns  turns          position
 *   g_gait_vel    turns/s        velocity at nominal playback
 *   g_gait_tau    Nm             joint torque the optimiser solved for
 *
 * FOUR columns, not five: this robot has no ankle roll, so there is no
 * GAIT_COL_ANKLE_ROLL and nothing may index one.
 */

{cols}

#define GAIT_SAMPLES     {n}
#define GAIT_JOINTS      {len(SLOTS)}
#define GAIT_DT_S        {T / n:.9f}f
#define GAIT_RATE_HZ     {n / T:.4f}f
#define GAIT_CYCLE_S     {T:.9f}f

extern const float g_gait_turns[GAIT_SAMPLES][GAIT_JOINTS];
extern const float g_gait_vel[GAIT_SAMPLES][GAIT_JOINTS];
extern const float g_gait_tau[GAIT_SAMPLES][GAIT_JOINTS];

/* Linear interpolation with wrap-around, exactly as in training. Any output may
   be NULL. Velocity and torque are at nominal playback (APP_GAIT_SPEED 1.0). */
void gait_sample(float phase, float *turns_out);
void gait_sample_all(float phase, float *turns_out, float *vel_out, float *tau_out);

/*
 * Position and velocity only - gait_sample_all with tau_out NULL.
 *
 * Kept because the single-leg bench test (test_leg_can.c) is where this API
 * started and still calls it. New code should call gait_sample_all: the torque
 * row is what the policy's feedforward needs, and a caller that cannot see it
 * has no way to know it is there.
 */
void gait_sample_vel(float phase, float *turns_out, float *vel_out);

#endif /* GAIT_REF_H */
"""

    c = f"""#include "gait_ref.h"

/* Generated by tools/gen_gait_from_library.py from {run}. Do not edit. */
{table('g_gait_turns', q[:, idx] / tp)}
{table('g_gait_vel', dq[:, idx] / tp)}
{table('g_gait_tau', tau[:, idx])}
static void lerp(const float t[GAIT_SAMPLES][GAIT_JOINTS], int i, int j, float f, float *out)
{{
    if (out == 0)
    {{
        return;
    }}
    for (int k = 0; k < GAIT_JOINTS; k++)
    {{
        out[k] = t[i][k] + (t[j][k] - t[i][k]) * f;
    }}
}}

void gait_sample_all(float phase, float *turns_out, float *vel_out, float *tau_out)
{{
    /* Wrap into [0,1). The phase clock is free-running. */
    phase -= (float)(int)phase;
    if (phase < 0.0f)
    {{
        phase += 1.0f;
    }}

    float x = phase * (float)GAIT_SAMPLES;
    int   i = (int)x;
    float f = x - (float)i;

    if (i >= GAIT_SAMPLES)
    {{
        i = GAIT_SAMPLES - 1;
        f = 1.0f;
    }}

    int j = (i + 1) % GAIT_SAMPLES;

    lerp(g_gait_turns, i, j, f, turns_out);
    lerp(g_gait_vel, i, j, f, vel_out);
    lerp(g_gait_tau, i, j, f, tau_out);
}}

void gait_sample(float phase, float *turns_out)
{{
    gait_sample_all(phase, turns_out, 0, 0);
}}

void gait_sample_vel(float phase, float *turns_out, float *vel_out)
{{
    gait_sample_all(phase, turns_out, vel_out, 0);
}}
"""
    return h, c, dict(row=row, T=T, n=n, speed=speed)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lib", default=os.path.join(ROOT, "tools", "model",
                                                   "gait_library.npz"),
                    help="gait_library.npz (default: the vendored copy)")
    ap.add_argument("--speed", type=float, default=0.30)
    ap.add_argument("--run", default="unknown", help="for the provenance comment")
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if the committed files differ, write nothing")
    a = ap.parse_args(argv)

    h, c, info = build(a.lib, a.speed, a.run)
    paths = {"gait_ref.h": h, "gait_ref.c": c}

    if a.check:
        stale = [n for n, text in paths.items()
                 if not os.path.exists(os.path.join(APP, n))
                 or open(os.path.join(APP, n)).read() != text]
        for n in stale:
            print("stale:", os.path.join("Appli/App", n))
        if stale:
            return 1
        print(f"up to date with {a.lib} row v={a.speed}")
        return 0

    for n, text in paths.items():
        open(os.path.join(APP, n), "w").write(text)
        print("wrote", os.path.join("Appli/App", n))
    print(f"  row {info['row']}, v = {info['speed']:.2f} m/s, "
          f"stride {info['T']:.6f} s, {info['n']} samples, {len(SLOTS)} columns")
    return 0


if __name__ == "__main__":
    sys.exit(main())
