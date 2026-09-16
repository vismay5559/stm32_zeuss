#!/usr/bin/env python3
"""
Compare the estimator's output with the simulated truth.

    python3 tools/sim/evaluate.py OUT_DIR [--plot] [--check]

Reads OUT_DIR/truth.npz (from gen_walk.py) and OUT_DIR/estimate.bin (from
replay.c), and reports only errors that mean something for this filter:

  tilt       angle between the true and estimated "up" direction, seen from
             the body. Roll and pitch are observable, so this should be small.
  velocity   error of the velocity in the BODY frame. Yaw is not observable
             and drifts; comparing world-frame velocity would charge the
             filter for a heading it cannot know, the body frame does not.
  yaw        reported, not judged: it drifts by design.
  height     estimated minus true IMU height above the starting ground.
             Absolute height is only weakly observable - the anchor is the
             first contact - so it may wander slowly.
  foot_z     reported foot height (lowest of toe and heel) against the truth.
  biases     estimated against the constant biases that were injected.

--check exits 1 if any error is beyond LIMITS below. The limits are about
two to three times what the default walk gives today: loose enough not to
flake on a change of noise seed, tight enough that a real mistake fails it.
Checked by corrupting the simulated sensors: a flipped knee sign, the spring
deflections dropped or marked invalid, and a 3 degree waist zero error all
fail. Toe and heel switches swapped does NOT - it makes velocity 50% worse,
but the filter's foot-slip allowance absorbs the few centimetres a rolling
foot moves, so it stays inside the limits.

Needs numpy; --plot also needs matplotlib and writes OUT_DIR/estimate.png.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

OUT_FIELDS = (["t_us", "status", "contacts_used"] + [f"q{i}" for i in "wxyz"]
              + [f"p{i}" for i in "xyz"] + [f"v{i}" for i in "xyz"]
              + [f"bg{i}" for i in "xyz"] + [f"ba{i}" for i in "xyz"]
              + ["foot_z_r", "foot_z_l", "fk_valid", "vh_lat", "vh_fwd", "vh_up"])


# (metric, limit, unit) for --check, judged after the settle time.
LIMITS = [
    ("tilt rms", 0.5, "deg"),
    ("tilt max", 1.0, "deg"),
    ("body velocity rms", 0.03, "m/s"),
    ("body velocity max", 0.08, "m/s"),
    ("height rms", 0.025, "m"),
    ("height drift", 0.03, "m"),
    ("foot_z rms", 0.025, "m"),
]


def quat_to_R(q):
    w, x, y, z = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - w * z); R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y); R[:, 2, 1] = 2 * (y * z + w * x); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir")
    ap.add_argument("--settle", type=float, default=2.0, help="s ignored at the start while the filter converges")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--check", action="store_true", help="exit 1 if any error exceeds LIMITS")
    a = ap.parse_args(argv)

    T = np.load(os.path.join(a.out_dir, "truth.npz"))
    E = np.fromfile(os.path.join(a.out_dir, "estimate.bin"), dtype="<f8").reshape(-1, len(OUT_FIELDS))
    e = {name: E[:, i] for i, name in enumerate(OUT_FIELDS)}
    n = min(len(T["t"]), len(E))
    t = T["t"][:n]
    judged = t >= a.settle
    walking = (t >= float(T["t_walk"])) & (t < float(T["t_end"]))

    R_true = T["R"][:n]
    R_est = quat_to_R(np.column_stack([e["qw"], e["qx"], e["qy"], e["qz"]])[:n])

    up_true = R_true[:, 2, :]            # world z seen in body = last row of R
    up_est = R_est[:, 2, :]
    tilt = np.degrees(np.arccos(np.clip(np.sum(up_true * up_est, axis=1)
                                        / np.linalg.norm(up_est, axis=1), -1, 1)))

    yaw_true = np.arctan2(R_true[:, 1, 0], R_true[:, 0, 0])
    yaw_est = np.arctan2(R_est[:, 1, 0], R_est[:, 0, 0])
    yaw_err = np.degrees(np.angle(np.exp(1j * (yaw_est - yaw_true))))

    v_est_w = np.column_stack([e["vx"], e["vy"], e["vz"]])[:n]
    vb_true = np.einsum("nji,nj->ni", R_true, T["v"][:n])
    vb_est = np.einsum("nji,nj->ni", R_est, v_est_w)
    verr = vb_est - vb_true

    h_err = e["pz"][:n] - T["p"][:n, 2]
    fz_err = np.column_stack([e["foot_z_r"], e["foot_z_l"]])[:n] - T["foot_z"][:n]

    bg_est = np.column_stack([e["bgx"], e["bgy"], e["bgz"]])[:n]
    ba_est = np.column_stack([e["bax"], e["bay"], e["baz"]])[:n]

    status = e["status"][:n].astype(int)
    first_conv = t[np.argmax(status >= 1)] if np.any(status >= 1) else float("nan")

    def row(label, x, unit, fmt="{:.3f}"):
        xj = x[judged]
        xw = x[judged & walking]
        print(f"  {label:<22} rms {fmt.format(rms(xj)):>8}  max {fmt.format(np.abs(xj).max()):>8}"
              f"   walking rms {fmt.format(rms(xw)):>8}  {unit}")

    dist = np.linalg.norm(T["p"][n - 1, :2] - T["p"][0, :2])
    print(f"simulated {t[-1]:.1f} s, {dist:.2f} m walked; judged after {a.settle:.1f} s")
    print(f"  status: CONVERGING from {first_conv:.2f} s; "
          f"contacts used while walking {np.unique(e['contacts_used'][:n][walking]).astype(int).tolist()}")
    row("tilt (roll/pitch)", tilt, "deg")
    row("body velocity |err|", np.linalg.norm(verr, axis=1), "m/s")
    for i, ax in enumerate("xyz"):
        row(f"  body v{ax}", verr[:, i], "m/s")
    row("height", h_err, "m")
    row("foot_z right", fz_err[:, 0], "m")
    row("foot_z left", fz_err[:, 1], "m")
    print(f"  {'yaw (unobservable)':<22} end {yaw_err[-1]:+.2f} deg")
    print(f"  {'height drift':<22} end {h_err[-1]:+.4f} m after {dist:.2f} m")
    print(f"  gyro bias   true {np.round(T['gyro_bias'], 4)}  est {np.round(bg_est[-1], 4)}")
    print(f"  accel bias  true {np.round(T['accel_bias'], 3)}  est {np.round(ba_est[-1], 3)}")

    finite = np.all(np.isfinite(E[:n, 3:16]))
    got = {
        "tilt rms": rms(tilt[judged]),
        "tilt max": float(tilt[judged].max()),
        "body velocity rms": rms(np.linalg.norm(verr, axis=1)[judged]),
        "body velocity max": float(np.linalg.norm(verr, axis=1)[judged].max()),
        "height rms": rms(h_err[judged]),
        "height drift": abs(float(h_err[-1])),
        "foot_z rms": rms(fz_err[judged]),
    }

    failed = []
    if a.check:
        if not finite:
            failed.append("NaN or inf in the estimate")
        for name, limit, unit in LIMITS:
            if not got[name] <= limit:
                failed.append(f"{name} {got[name]:.4f} {unit} > {limit} {unit}")
        print("check:", "FAILED\n  " + "\n  ".join(failed) if failed else "PASSED")

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(5, 1, figsize=(11, 13), sharex=True)
        ax[0].plot(t, tilt); ax[0].set_ylabel("tilt err (deg)")
        for i, ax_name in enumerate("xyz"):
            ax[1].plot(t, vb_true[:, i], label=f"true v{ax_name}", lw=1)
            ax[1].plot(t, vb_est[:, i], "--", label=f"est v{ax_name}", lw=1)
        ax[1].set_ylabel("body vel (m/s)"); ax[1].legend(ncol=6, fontsize=7)
        ax[2].plot(t, T["p"][:n, 2], label="true"); ax[2].plot(t, e["pz"][:n], "--", label="est")
        ax[2].set_ylabel("IMU height (m)"); ax[2].legend(fontsize=7)
        ax[3].plot(t, T["foot_z"][:n, 1], label="true left"); ax[3].plot(t, e["foot_z_l"][:n], "--", label="est left")
        ax[3].plot(t, T["foot_z"][:n, 0], label="true right"); ax[3].plot(t, e["foot_z_r"][:n], "--", label="est right")
        ax[3].set_ylabel("foot z (m)"); ax[3].legend(ncol=4, fontsize=7)
        ax[4].plot(t, yaw_err, label="yaw err (deg)"); ax[4].plot(t, status, label="status")
        ax[4].plot(t, e["contacts_used"][:n], label="contacts"); ax[4].legend(fontsize=7)
        ax[4].set_xlabel("t (s)")
        for x in ax:
            x.axvspan(float(T["t_walk"]), float(T["t_end"]), color="0.9", zorder=-1)
        fig.tight_layout()
        path = os.path.join(a.out_dir, "estimate.png")
        fig.savefig(path, dpi=110)
        print("plot:", path)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
