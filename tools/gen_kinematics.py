#!/usr/bin/env python3
"""
zeus.urdf  ->  Appli/App/zeus_kinematics_model.h: the estimator's leg geometry.

The contact-aided InEKF needs, every tick, where each toe and heel contact point
is relative to the IMU, and how that point moves with each joint (the Jacobian,
which turns encoder noise into measurement noise). That used to be written by
hand around four measured lengths. Now the geometry comes from the CAD model:

    zeus_26/zeus_description/urdf/zeus.urdf          (made from the Fusion export)
        │
        ├─ this script walks the URDF from imu_link to each toe and heel,
        │  folds every rigid part between two joints into one constant
        │  transform, and writes the result as C tables
        │                    ─► Appli/App/zeus_kinematics_model.h
        │                       read by zeus_kinematics.c, which does FK and the
        │                       exact Jacobian on the STM32
        │
        └─ Pinocchio, an independent implementation, in double precision
                             ─► tools/hosttest/zeus_kinematics_ref.h
                                answers tools/hosttest/test_zeus_kinematics.c
                                holds the firmware's float code to

Why tables and not symbolic code generation (CasADi): the CAD's transforms are
full of float noise (6e-17, 2e-19, joint axes a hair off true), which stops a
symbolic tool simplifying anything. Tried on this model it produced ~5000
operations and 400 KB of C per contact point; the table walk is a few hundred
operations for both points and their Jacobians together.

Nothing here runs on the robot. Re-run whenever zeus.urdf changes and commit
what it writes.

Setup, once, in a venv (never pip --user on the ROS machine):

    python3 -m venv ~/kin_venv
    ~/kin_venv/bin/pip install -r tools/requirements-kinematics.txt

Run from the repo root. Unset PYTHONPATH if ROS is sourced - its eigenpy
clashes with Pinocchio's:

    env -u PYTHONPATH ~/kin_venv/bin/python tools/gen_kinematics.py
    env -u PYTHONPATH ~/kin_venv/bin/python tools/gen_kinematics.py --urdf other/zeus.urdf
    env -u PYTHONPATH ~/kin_venv/bin/python tools/gen_kinematics.py --check   # exit 1 if stale
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata as md
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, "Appli", "App")
HOSTTEST = os.path.join(ROOT, "tools", "hosttest")
DEFAULT_URDF = os.path.join(os.path.dirname(ROOT), "zeus_26", "zeus_description", "urdf", "zeus.urdf")

# q, per leg. Must match the ZEUS_KIN_Q_* enum in zeus_kinematics.h (checked
# below). Fixed by name, not by the URDF's nesting, so a re-export of the real
# robot - hip pitch as the parent - changes the tables, never the meaning of q.
Q = [
    ("HIP_PITCH", "{s}_hip_pitch"),
    ("HIP_ROLL", "{s}_hip_roll"),
    ("KNEE_PITCH", "{s}_knee_pitch"),
    ("ANKLE_PITCH", "{s}_ankle_pitch"),
    ("HIP_PITCH_SPRING", "{s}_hip_pitch_spring"),
    ("KNEE_PITCH_SPRING", "{s}_knee_pitch_spring"),
    ("WAIST_PITCH", "waist_pitch"),
    ("WAIST_ROLL", "waist_roll"),
]
SIDES = ["left", "right"]              # ZEUS_KIN_LEFT, ZEUS_KIN_RIGHT
POINTS = ["toe", "heel"]               # ZEUS_KIN_TOE, ZEUS_KIN_HEEL
IMU = "imu_link"
NQ = len(Q)

FILES = {
    "model": os.path.join(APP, "zeus_kinematics_model.h"),
    "ref": os.path.join(HOSTTEST, "zeus_kinematics_ref.h"),
}
N_REF = 24                             # poses per leg in the reference set


# ---------------------------------------------------------------- URDF

def rpy_to_R(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def vec(s, default=(0.0, 0.0, 0.0)):
    return np.array([float(x) for x in s.split()]) if s else np.array(default, float)


class Urdf:
    def __init__(self, text: str):
        root = ET.fromstring(text)
        self.links = {l.get("name") for l in root.findall("link")}
        self.by_child = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            T = np.eye(4)
            if o is not None:
                T[:3, :3] = rpy_to_R(vec(o.get("rpy")))
                T[:3, 3] = vec(o.get("xyz"))
            ax = j.find("axis")
            a = vec(ax.get("xyz") if ax is not None else "", (1.0, 0.0, 0.0))
            self.by_child[j.find("child").get("link")] = {
                "name": j.get("name"), "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "T": T, "axis": a / np.linalg.norm(a),
            }

    def up(self, link: str):
        """Joints from `link` up to the root, nearest first."""
        out = []
        while link in self.by_child:
            j = self.by_child[link]
            out.append(j)
            link = j["parent"]
        return out

    def path(self, start: str, end: str):
        """(joint, +1 parent->child | -1 child->parent) from `start` to `end`."""
        for n in (start, end):
            if n not in self.links:
                raise SystemExit(f"the URDF has no link named {n}")
        a, b = self.up(start), self.up(end)
        common = 0
        while (common < min(len(a), len(b)) and
               a[len(a) - 1 - common]["name"] == b[len(b) - 1 - common]["name"]):
            common += 1
        return ([(j, -1) for j in a[:len(a) - common]] +
                [(j, +1) for j in reversed(b[:len(b) - common])])


def leg_tables(urdf: Urdf, side: str):
    """Steps (pre_T, axis, sign, q index) and the toe/heel points, for one leg."""
    qidx = {name.format(s=side): i for i, (_, name) in enumerate(Q)}
    legs = []
    for point in POINTS:
        steps, C = [], np.eye(4)
        for j, direction in urdf.path(IMU, f"{side}_{point}"):
            moving = j["type"] in ("revolute", "continuous")
            if j["type"] not in ("fixed", "revolute", "continuous"):
                raise SystemExit(f"{j['name']}: {j['type']} joints are not supported")
            if moving and j["name"] not in qidx:
                raise SystemExit(f"{j['name']} moves the {side} {point} but is not in the q vector")
            if direction > 0:
                C = C @ j["T"]
                if moving:
                    steps.append((C, j["axis"], 1.0, qidx[j["name"]]))
                    C = np.eye(4)
            else:
                if moving:
                    steps.append((C, j["axis"], -1.0, qidx[j["name"]]))
                    C = np.eye(4)
                C = C @ np.linalg.inv(j["T"])
        legs.append((steps, C[:3, 3]))

    (steps, toe), (steps_heel, heel) = legs
    same = len(steps) == len(steps_heel) and all(
        np.allclose(a[0], b[0], atol=1e-12) and a[2:] == b[2:] for a, b in zip(steps, steps_heel))
    if not same:
        raise SystemExit(f"{side} toe and heel are not on the same link chain")
    used = sorted(s[3] for s in steps)
    if used != list(range(NQ)):
        missing = [Q[i][1].format(s=side) for i in range(NQ) if i not in used]
        raise SystemExit(f"{side} leg: {missing or 'a joint'} not exactly once on the path IMU -> foot")
    return steps, [toe, heel]


# ---------------------------------------------------------------- output

def provenance(urdf_path: str, text: str) -> dict:
    vers = []
    for pkg in ("pin", "numpy"):
        try:
            vers.append(f"{pkg} {md.version(pkg)}")
        except md.PackageNotFoundError:
            vers.append(f"{pkg} ?")
    parent = os.path.dirname(ROOT) + os.sep
    return {"sha": hashlib.sha256(text.encode()).hexdigest(),
            "name": urdf_path[len(parent):] if urdf_path.startswith(parent) else os.path.basename(urdf_path),
            "tools": ", ".join(vers)}


def banner(prov: dict, what: str) -> str:
    return ("/*\n"
            " * GENERATED by tools/gen_kinematics.py - do not edit, re-run it.\n"
            " *\n"
            f" * {what}\n"
            " *\n"
            f" * model   {prov['name']}\n"
            f" * sha256  {prov['sha']}\n"
            f" * tools   {prov['tools']}\n"
            " */\n")


def f32(v) -> str:
    def one(x):
        if abs(x) < 1e-12:
            return "0.0f"
        s = f"{float(x):.9g}"
        return (s if ("." in s or "e" in s) else s + ".0") + "f"
    return ", ".join(one(x) for x in np.ravel(v))


def f64(v) -> str:
    return ", ".join(f"{float(x):.12g}" for x in np.ravel(v))


def model_header(prov: dict, tables, zero_pose) -> str:
    legs = []
    for side, (steps, points) in zip(SIDES, tables):
        rows = []
        for C, axis, sign, qi in steps:
            rows.append(f"            {{ /* {Q[qi][1].format(s=side)}{' (walked child to parent)' if sign < 0 else ''} */\n"
                        f"              .pre_R = {{ {f32(C[:3, :3])} }},\n"
                        f"              .pre_p = {{ {f32(C[:3, 3])} }},\n"
                        f"              .axis  = {{ {f32(axis)} }},\n"
                        f"              .sign  = {sign:.1f}f, .q = ZEUS_KIN_Q_{Q[qi][0]} }},")
        legs.append(f"    [ZEUS_KIN_{side.upper()}] = {{\n"
                    f"        .step = {{\n" + "\n".join(rows) + "\n        },\n"
                    f"        .point = {{ {{ {f32(points[0])} }},    /* toe  */\n"
                    f"                   {{ {f32(points[1])} }} }},  /* heel */\n"
                    f"    }},")
    zp = "\n".join(f" *   {s:<5} {pt:<4}  ({v[0]:+.4f}, {v[1]:+.4f}, {v[2]:+.4f})"
                   for (s, pt), v in zero_pose.items())
    return banner(prov, "Leg geometry for zeus_kinematics.c. Include it from there only.") + f"""
#ifndef ZEUS_KINEMATICS_MODEL_H
#define ZEUS_KINEMATICS_MODEL_H

#include "zeus_kinematics.h"

/*
 * Contact points in the IMU frame at q = 0 (m), as a sanity check against
 * the robot in front of you:
{zp}
 */

#define ZK_STEPS  {len(tables[0][0])}

typedef struct
{{
    float pre_R[9];     /* rigid transform since the previous joint, row-major */
    float pre_p[3];
    float axis[3];      /* unit, in the joint's own frame                     */
    float sign;         /* -1 where the walk goes child to parent             */
    uint8_t q;          /* ZEUS_KIN_Q_*                                       */
}} zk_step_t;

typedef struct
{{
    zk_step_t step[ZK_STEPS];
    float point[ZEUS_KIN_POINTS][3];    /* toe, heel in the last joint's frame */
}} zk_leg_t;

static const zk_leg_t zk_legs[2] = {{
{chr(10).join(legs)}
}};

const char zeus_kin_model_sha[] = "{prov['sha'][:16]}";

#endif /* ZEUS_KINEMATICS_MODEL_H */
"""


def reference(urdf_path: str, prov: dict):
    import pinocchio as pin

    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    rng = np.random.default_rng(20260916)

    def contact(side, q8, point):
        qf = pin.neutral(model)
        for (_, name), v in zip(Q, q8):
            qf[model.joints[model.getJointId(name.format(s=side))].idx_q] = v
        pin.framesForwardKinematics(model, data, qf)
        return data.oMf[model.getFrameId(IMU)].actInv(data.oMf[model.getFrameId(f"{side}_{point}")].translation)

    poses = [np.zeros(NQ)]
    for _ in range(N_REF - 1):
        q = rng.uniform(-0.78, 0.78, NQ)         # inside the +/-45 deg limits
        q[4:6] = rng.uniform(-0.15, 0.15, 2)            # springs deflect a little
        q[6:8] = rng.uniform(-0.3, 0.3, 2)
        poses.append(q)

    h = 1e-6
    rows, zero_pose = [], {}
    for side in SIDES:
        for q in poses:
            for point in POINTS:
                p = contact(side, q, point)
                J = np.column_stack([(contact(side, q + h * e, point) - contact(side, q - h * e, point)) / (2 * h)
                                     for e in np.eye(NQ)])
                rows.append((side, point, q, p, J))
                if not q.any():
                    zero_pose[(side, point)] = p
    body = "\n".join(f"    {{ {SIDES.index(s)}, {POINTS.index(pt)},\n"
                     f"      {{ {f64(q)} }},\n"
                     f"      {{ {f64(p)} }},\n"
                     f"      {{ {f64(J)} }} }},"
                     for s, pt, q, p, J in rows)
    text = banner(prov, "Reference answers from Pinocchio: double precision, J by central differences.") + f"""
#ifndef ZEUS_KINEMATICS_REF_H
#define ZEUS_KINEMATICS_REF_H

typedef struct
{{
    int    side;            /* ZEUS_KIN_LEFT / RIGHT  */
    int    point;           /* ZEUS_KIN_TOE / HEEL    */
    double q[{NQ}];
    double p[3];            /* IMU frame, m           */
    double J[{3 * NQ}];           /* row-major 3 x {NQ}, m/rad */
}} zeus_kin_ref_t;

static const zeus_kin_ref_t zeus_kin_ref[] = {{
{body}
}};

#define ZEUS_KIN_REF_N  ((int)(sizeof zeus_kin_ref / sizeof zeus_kin_ref[0]))

#endif /* ZEUS_KINEMATICS_REF_H */
"""
    return text, zero_pose


def check_enum():
    h = open(os.path.join(APP, "zeus_kinematics.h")).read()
    enum = dict((m.group(1), int(m.group(2))) for m in re.finditer(r"ZEUS_KIN_Q_(\w+)\s*=\s*(\d+)", h))
    want = {e: i for i, (e, _) in enumerate(Q)}
    if enum != want:
        raise SystemExit(f"zeus_kinematics.h ZEUS_KIN_Q_* {enum} does not match this script's Q {want}")


def generate(urdf_path: str) -> dict:
    check_enum()
    text = open(urdf_path).read()
    prov = provenance(urdf_path, text)
    urdf = Urdf(text)
    tables = [leg_tables(urdf, side) for side in SIDES]
    ref, zero_pose = reference(urdf_path, prov)
    return {"model": model_header(prov, tables, zero_pose), "ref": ref}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=DEFAULT_URDF, help=f"default: {DEFAULT_URDF}")
    ap.add_argument("--check", action="store_true", help="exit 1 if the checked-in files are stale")
    a = ap.parse_args(argv)
    urdf = os.path.abspath(a.urdf)
    if not os.path.exists(urdf):
        raise SystemExit(f"no URDF at {urdf} - pass --urdf")

    out = generate(urdf)
    stale = [k for k, t in out.items() if not os.path.exists(FILES[k]) or open(FILES[k]).read() != t]
    if a.check:
        for k in stale:
            print(f"stale: {os.path.relpath(FILES[k], ROOT)}", file=sys.stderr)
        if not stale:
            print(f"up to date with {urdf}")
        return 1 if stale else 0
    for k, t in out.items():
        with open(FILES[k], "w", newline="\n") as f:
            f.write(t)
        print("wrote", os.path.relpath(FILES[k], ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
