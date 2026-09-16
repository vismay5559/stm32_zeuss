#!/usr/bin/env python3
"""
A simulated walk with exact ground truth, for testing the estimator.

The estimator's maths can only be judged against the true motion, and on the
robot nobody knows the true motion. So this makes one up that is exactly
consistent with the robot's own geometry (zeus.urdf):

  - The legs follow smooth joint trajectories: a step, a knee lift on the
    swing leg, a little hip-roll sway, waist wobble, spring wind-up on the
    stance leg.
  - Each foot lands heel first, toe raised, and rolls down flat; at the end of
    stance the heel lifts and it rolls over the toe. The switches follow:
    heel only, then both, then toe only. While a foot rolls, only the point it
    rolls about stays still - which is exactly what the estimator must get
    right, and what a flat-footed walk could never test.
  - Whatever is on the ground is bolted to the world. The torso's motion is
    whatever the joint angles make it, computed backwards from that - so the
    one thing the estimator assumes (a contact point whose switch is closed
    does not move) is exactly true, and everything else follows from the URDF.
  - At each handover every joint has zero velocity and acceleration, so the
    torso's motion stays smooth and the IMU sees no impacts.

From that motion it writes what the STM32's sensors would have reported -
BNO085 gyro and accelerometer at 400 Hz with noise and a constant bias, ODrive
positions in turns, AS5047P spring counts, foot switch bits - in the layout
tools/sim/replay.c feeds to the real fusion.c, and the truth next to it for
tools/sim/evaluate.py.

This is a KINEMATIC walk, not a physics simulation: nothing here balances or
falls over. That is the point - the test is of the estimator, and a physics
engine would only add its own contact model's errors to the truth.

Needs Pinocchio (tools/requirements-kinematics.txt):

    env -u PYTHONPATH ~/kin_venv/bin/python tools/sim/gen_walk.py
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_URDF = os.path.join(os.path.dirname(ROOT), "zeus_26", "zeus_description", "urdf", "zeus.urdf")
LINK_PROTO = os.path.join(ROOT, "Appli", "App", "link_proto.h")
OUT_DIR = os.path.join(ROOT, "tools", "sim", "out")

SIDES = ("left", "right")
POINTS = ("toe", "heel")
GRAVITY = 9.81
ENC_COUNTS = 16384

# Replay input record: every field a little-endian float64, in this order.
INPUT_FIELDS = (["t_us", "imu_new"] + [f"gyro{i}" for i in range(3)] + [f"accel{i}" for i in range(3)]
                + [f"pos{i}" for i in range(10)] + [f"pos_age{i}" for i in range(10)]
                + [f"enc{i}" for i in range(4)] + ["enc_valid", "contacts"])


def proto_indices():
    """Joint, encoder and contact indices, read from link_proto.h so they cannot drift."""
    text = open(LINK_PROTO).read()

    def macro(name):
        m = re.search(rf"#define\s+{name}\s+\(?\s*(1u\s*<<\s*)?(\d+)", text)
        if not m:
            raise SystemExit(f"{name} not found in link_proto.h")
        v = int(m.group(2))
        return (1 << v) if m.group(1) else v

    act = {}
    for side, s in (("left", "L"), ("right", "R")):
        for j in ("HIP_PITCH", "HIP_ROLL", "KNEE_PITCH", "ANKLE_PITCH"):
            act[f"{side}_{j.lower()}"] = macro(f"NEXUS_J_{s}_{j}")
    act["waist_pitch"] = macro("NEXUS_J_WAIST_PITCH")
    act["waist_roll"] = macro("NEXUS_J_WAIST_ROLL")
    enc = {f"{side}_{j.lower()}_spring": macro(f"NEXUS_ENC_{s}_{j}")
           for side, s in (("left", "L"), ("right", "R")) for j in ("HIP_PITCH", "KNEE_PITCH")}
    bit = {(side, pt): macro(f"NEXUS_CONTACT_{s}_{pt.upper()}_BIT")
           for side, s in (("left", "L"), ("right", "R")) for pt in POINTS}
    foot = {"left": macro("NEXUS_CONTACT_L_FOOT"), "right": macro("NEXUS_CONTACT_R_FOOT")}
    return act, enc, bit, foot


# ---------------------------------------------------------------- the gait

def quintic(x):
    """0 -> 1 with zero velocity and acceleration at both ends."""
    x = min(max(x, 0.0), 1.0)
    return x * x * x * (10.0 - 15.0 * x + 6.0 * x * x)


def ramp(s, lo, hi):
    """quintic over the part of the step between lo and hi."""
    return quintic((s - lo) / (hi - lo))


class Gait:
    def __init__(self, a):
        self.a = a
        self.t_walk = a.stand
        self.n = a.steps
        self.t_end = a.stand + a.steps * a.step_time
        self.duration = self.t_end + a.stand_end

    def segment(self, t):
        """Which step this time belongs to, clamped so standing uses the first or last."""
        i = int(math.floor((t - self.t_walk) / self.a.step_time))
        return min(max(i, 0), self.n - 1)

    def stance(self, i):
        return SIDES[i % 2]

    def phase(self, i, t):
        return quintic((t - (self.t_walk + i * self.a.step_time)) / self.a.step_time)

    # Foot pitch against flat, about +Y: positive = heel up (rolling over the
    # toe), negative = toe up (rolling onto the heel). The first step starts
    # from flat standing and the last lands flat for standing again.
    def heel_strike(self, i):
        return self.a.roll if i > 0 else 0.0

    def toe_off(self, i):
        return self.a.roll if i < self.n - 1 else 0.0

    def stance_pitch(self, i, s):
        heel = -self.heel_strike(i) * (1.0 - ramp(s, 0.0, self.a.heel_phase))
        toe = self.toe_off(i) * ramp(s, self.a.toe_phase, 1.0)
        return heel, toe

    def swing_pitch(self, i, s):
        start = self.toe_off(i - 1) if i > 0 else 0.0      # it just rolled off its toe
        end = -self.heel_strike(i + 1) if i < self.n - 1 else 0.0
        return start + (end - start) * quintic(s)

    def stance_switches(self, i, s):
        """Which of the stance foot's switches are closed."""
        if self.heel_strike(i) and s < self.a.heel_phase:
            return ("heel",)
        if self.toe_off(i) and s > self.a.toe_phase:
            return ("toe",)
        return POINTS

    def q(self, i, s):
        a = self.a
        st = self.stance(i)
        sw = SIDES[1 - SIDES.index(st)]
        u = 2.0 * s - 1.0
        lift = math.sin(math.pi * s)
        q = {}
        # The stance hip extends (foot passes under and behind), the swing hip flexes forward.
        q[f"{st}_hip_pitch"] = a.hip0 + a.hip_amp * u
        q[f"{sw}_hip_pitch"] = a.hip0 - a.hip_amp * u
        pitch = {st: sum(self.stance_pitch(i, s)), sw: self.swing_pitch(i, s)}
        for side in SIDES:
            knee = a.knee0 + (a.knee_lift * lift * lift if side == sw else 0.0)
            loaded = 1.0 if side == st else 0.0
            q[f"{side}_knee_pitch"] = knee
            q[f"{side}_hip_pitch_spring"] = 0.4 * a.spring * lift * loaded
            q[f"{side}_knee_pitch_spring"] = a.spring * lift * loaded
            # Foot parallel to the pelvis - springs included, as a controller
            # would hold it - plus the roll: an ankle that turns the foot by the
            # same angle it pivots on the ground keeps the pelvis level.
            leg_pitch = (q[f"{side}_hip_pitch"] + q[f"{side}_hip_pitch_spring"]
                         + knee + q[f"{side}_knee_pitch_spring"])
            q[f"{side}_ankle_pitch"] = -leg_pitch + pitch[side]
            q[f"{side}_hip_roll"] = a.sway * lift * (1.0 if st == "left" else -1.0)
        q["waist_pitch"] = a.waist_pitch * math.sin(2.0 * math.pi * s)
        q["waist_roll"] = a.waist_roll * math.sin(2.0 * math.pi * s)
        return q


# ---------------------------------------------------------------- kinematics

class Robot:
    def __init__(self, urdf):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.fid = {n: self.model.getFrameId(n) for n in
                    ["imu_link", "left_foot", "right_foot"] + [f"{s}_{p}" for s in SIDES for p in POINTS]}

    def frames(self, qd):
        pin = self.pin
        q = pin.neutral(self.model)
        for name, v in qd.items():
            q[self.model.joints[self.model.getJointId(name)].idx_q] = v
        pin.framesForwardKinematics(self.model, self.data, q)
        return {n: self.data.oMf[i].homogeneous.copy() for n, i in self.fid.items()}


def inv(T):
    R, p = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ p
    return out


def pivot(c, theta):
    """Rotation by theta about the world Y line through point c."""
    T = np.eye(4)
    co, si = math.cos(theta), math.sin(theta)
    R = np.array([[co, 0.0, si], [0.0, 1.0, 0.0], [-si, 0.0, co]])
    T[:3, :3] = R
    T[:3, 3] = c - R @ c
    return T


def vee_skew(M):
    return np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]]) * 0.5


# ---------------------------------------------------------------- main

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--out", default=OUT_DIR)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--stand", type=float, default=3.0, help="s standing still before walking")
    ap.add_argument("--stand-end", type=float, default=2.0, help="s standing still after")
    ap.add_argument("--steps", type=int, default=16)
    ap.add_argument("--step-time", type=float, default=0.6)
    ap.add_argument("--hip0", type=float, default=None,
                    help="neutral hip pitch; default: the angle that lands both feet level")
    ap.add_argument("--hip-amp", type=float, default=0.20)
    ap.add_argument("--knee0", type=float, default=0.45)
    ap.add_argument("--knee-lift", type=float, default=0.30)
    ap.add_argument("--sway", type=float, default=0.04)
    ap.add_argument("--spring", type=float, default=0.06, help="rad of knee spring wind-up at mid-stance")
    ap.add_argument("--roll", type=float, default=0.15, help="rad of heel-strike and toe-off foot roll")
    ap.add_argument("--heel-phase", type=float, default=0.15, help="part of stance on the heel alone")
    ap.add_argument("--toe-phase", type=float, default=0.80, help="part of stance after which only the toe is down")
    ap.add_argument("--waist-pitch", type=float, default=0.05)
    ap.add_argument("--waist-roll", type=float, default=0.03)
    ap.add_argument("--gyro-noise", type=float, default=0.005, help="rad/s per sample")
    ap.add_argument("--accel-noise", type=float, default=0.03, help="m/s^2 per sample")
    ap.add_argument("--gyro-bias", type=float, nargs=3, default=[0.002, -0.003, 0.001])
    ap.add_argument("--accel-bias", type=float, nargs=3, default=[0.03, -0.02, 0.05])
    ap.add_argument("--drive-noise", type=float, default=0.001, help="rad per sample")
    ap.add_argument("--drive-period", type=int, default=2, help="ticks between ODrive position reports")
    a = ap.parse_args(argv)

    act_idx, enc_idx, bit, foot_bit = proto_indices()
    rng = np.random.default_rng(a.seed)
    robot = Robot(a.urdf)

    if a.hip0 is None:
        # At a handover one leg is forward and one back. Unless the neutral hip
        # angle is chosen for this knee bend and these link lengths, the back
        # leg reaches lower than the front one and every step lands higher
        # than the last - a staircase, not flat ground. Bisect for level feet.
        def level_error(h0):
            a.hip0 = h0
            f = robot.frames(Gait(a).q(0, 0.0))
            return min(f[f"left_{pt}"][2, 3] for pt in POINTS) - min(f[f"right_{pt}"][2, 3] for pt in POINTS)
        lo, hi = -0.6, 0.2
        if np.sign(level_error(lo)) == np.sign(level_error(hi)):
            raise SystemExit("could not find a hip angle that lands both feet level; pass --hip0")
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if np.sign(level_error(mid)) == np.sign(level_error(lo)):
                lo = mid
            else:
                hi = mid
        a.hip0 = 0.5 * (lo + hi)
        print(f"neutral hip pitch {a.hip0:.4f} rad lands both feet level")

    gait = Gait(a)

    # ---- where each step's stance foot is in the world
    #
    # A[i] is the stance foot's pose as it would be lying flat. The heel strike
    # pivots it about the heel, the toe-off about the toe, both about world Y
    # through that point - so the point doing the pivoting never moves.
    f0 = robot.frames(gait.q(0, 0.0))
    T_Fb = {side: inv(f0[f"{side}_foot"]) for side in SIDES}
    local = {(side, pt): (T_Fb[side] @ f0[f"{side}_{pt}"])[:3, 3] for side in SIDES for pt in POINTS}

    T_wb0 = np.eye(4)
    T_wb0[2, 3] = -f0["left_toe"][2, 3]                   # left toe on z = 0, torso upright

    A = []

    def foot_world(i, s):
        st = gait.stance(i)
        heel, toe = gait.stance_pitch(i, s)
        c_heel = (A[i] @ np.append(local[(st, "heel")], 1.0))[:3]
        c_toe = (A[i] @ np.append(local[(st, "toe")], 1.0))[:3]
        return pivot(c_heel, heel) @ pivot(c_toe, toe) @ A[i]

    T_wb = T_wb0
    for i in range(gait.n):
        st = gait.stance(i)
        T_wF = T_wb @ robot.frames(gait.q(i, 0.0))[f"{st}_foot"]
        c_heel = (T_wF @ np.append(local[(st, "heel")], 1.0))[:3]
        A.append(pivot(c_heel, gait.heel_strike(i)) @ T_wF)     # undo the landing roll
        T_wb = foot_world(i, 1.0) @ inv(robot.frames(gait.q(i, 1.0))[f"{st}_foot"])

    def at(t, i):
        s = gait.phase(i, t)
        q = gait.q(i, s)
        f = robot.frames(q)
        T_wb = foot_world(i, s) @ inv(f[f"{gait.stance(i)}_foot"])
        return q, T_wb, f, s

    # ---- sample
    dt = 1e-3
    n = int(round(gait.duration / dt))
    h = 1e-4
    g_w = np.array([0.0, 0.0, -GRAVITY])
    bg = np.array(a.gyro_bias)
    ba = np.array(a.accel_bias)

    rec = np.zeros((n, len(INPUT_FIELDS)))
    truth = {k: np.zeros(s) for k, s in {
        "t": (n,), "p": (n, 3), "R": (n, 3, 3), "v": (n, 3), "omega_b": (n, 3), "f_b": (n, 3),
        "foot_z": (n, 2), "contacts": (n,)}.items()}

    pos_turns = np.zeros(10)
    pos_age = np.zeros(10)
    next_imu = 0.0

    for k in range(n):
        t = k * dt
        i = gait.segment(t)
        q, T_wb, f, s = at(t, i)
        T_wi = T_wb @ f["imu_link"]
        _, T_wb_p, f_p, _ = at(t + h, i)
        _, T_wb_m, f_m, _ = at(t - h, i)
        T_p = T_wb_p @ f_p["imu_link"]
        T_m = T_wb_m @ f_m["imu_link"]

        R, p = T_wi[:3, :3], T_wi[:3, 3]
        v = (T_p[:3, 3] - T_m[:3, 3]) / (2 * h)
        acc = (T_p[:3, 3] - 2 * p + T_m[:3, 3]) / (h * h)
        Rdot = (T_p[:3, :3] - T_m[:3, :3]) / (2 * h)
        omega_b = vee_skew(R.T @ Rdot)
        f_b = R.T @ (acc - g_w)                               # what an accelerometer reads

        truth["t"][k] = t
        truth["p"][k], truth["R"][k], truth["v"][k] = p, R, v
        truth["omega_b"][k], truth["f_b"][k] = omega_b, f_b
        for s_i, side in enumerate(("right", "left")):        # foot_z order: [0] right, [1] left
            truth["foot_z"][k, s_i] = min((T_wb @ f[f"{side}_{pt}"])[2, 3] for pt in POINTS)

        # contacts: both feet flat while standing; the stance foot's switches
        # (heel, both, toe) while walking
        if t < gait.t_walk or t >= gait.t_end:
            down = [(side, pt) for side in SIDES for pt in POINTS]
        else:
            down = [(gait.stance(i), pt) for pt in gait.stance_switches(i, s)]
        contacts = 0
        for side, pt in down:
            contacts |= bit[(side, pt)] | foot_bit[side]
        truth["contacts"][k] = contacts

        # IMU at 400 Hz: a sample becomes visible on the first tick at or after it
        imu_new = 0
        gyro = accel = np.zeros(3)
        if t + 1e-9 >= next_imu:
            imu_new = 1
            next_imu += 0.0025
            gyro = omega_b + bg + rng.normal(0, a.gyro_noise, 3)
            accel = f_b + ba + rng.normal(0, a.accel_noise, 3)

        # drives: output-side turns, reported every drive_period ticks
        if k % a.drive_period == 0:
            for name, idx in act_idx.items():
                pos_turns[idx] = (q[name] + rng.normal(0, a.drive_noise)) / (2 * math.pi)
            pos_age[:] = 0
        else:
            pos_age += 1

        # springs: raw AS5047P counts, zero 0 and sign +1 as in robot_config.c
        enc = np.zeros(4)
        for name, idx in enc_idx.items():
            enc[idx] = int(round(q[name] / (2 * math.pi) * ENC_COUNTS)) % ENC_COUNTS

        rec[k] = ([t * 1e6, imu_new] + list(gyro) + list(accel) + list(pos_turns) + list(pos_age)
                  + list(enc) + [0x0F, contacts])

    os.makedirs(a.out, exist_ok=True)
    rec.astype("<f8").tofile(os.path.join(a.out, "input.bin"))
    np.savez(os.path.join(a.out, "truth.npz"), gyro_bias=bg, accel_bias=ba,
             t_walk=gait.t_walk, t_end=gait.t_end, **truth)

    dist = np.linalg.norm(truth["p"][-1, :2] - truth["p"][0, :2])
    Rz = truth["R"]
    tilt = np.degrees(np.arccos(np.clip(Rz[:, 2, 2], -1, 1)))
    print(f"torso tilt up to {tilt.max():.2f} deg, heel/toe roll {math.degrees(a.roll):.1f} deg")
    print(f"wrote {n} ticks ({gait.duration:.1f} s): {a.steps} steps, torso travelled {dist:.2f} m, "
          f"height {truth['p'][:, 2].min():.3f}..{truth['p'][:, 2].max():.3f} m, "
          f"peak accel {np.abs(truth['f_b'] - truth['f_b'][:1]).max():.1f} m/s^2")


if __name__ == "__main__":
    sys.exit(main())
