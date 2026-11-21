#!/usr/bin/env python3
"""Allan deviation of a still IMU log, and the InEKF noise settings it implies.

    python3 tools/imu_allan/allan.py imu_still.bin [--skip 2] [--stop 58] [--png out.png]

WHAT IT READS OFF THE PLOT, per axis:
  N  white noise density - the -1/2 slope line, read at tau = 1 s.
     gyro: rad/s/sqrt(Hz) (= rad/sqrt(s), angle random walk)
     accel: m/s^2/sqrt(Hz) (velocity random walk)
  B  bias instability - the flat bottom, divided by 0.664.
  K  rate random walk - the +1/2 slope line, read at tau = 3 s. Often there is
     no +1/2 region inside an hour; then K is reported as an upper bound from
     the last point, and says so.

HOW THEY MAP TO inekf_params_t (inekf.c adds Qbar * dt, so every field is a
continuous-time density, which is exactly what N and K are):
  noise_gyro        = N_gyro           noise_accel       = N_accel
  noise_gyro_bias   = K_gyro           noise_accel_bias  = K_accel
Recommended values are the WORST axis, then rounded UP: an estimator told the
IMU is quieter than it is trusts it over the legs and drifts confidently.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imu_frames import FRAME_HZ, decode  # noqa: E402


def adev(x, rate, n_taus=60):
    """Overlapping Allan deviation via cumulative sums."""
    n = len(x)
    dt = 1.0 / rate
    m = np.unique(np.logspace(0, np.log10(n // 9), n_taus).astype(int))
    theta = np.concatenate(([0.0], np.cumsum(x) * dt))
    out = []
    for mm in m:
        d = theta[2 * mm:] - 2 * theta[mm:-mm] + theta[:-2 * mm]
        out.append(np.sqrt(np.mean(d * d) / (2 * (mm * dt) ** 2)))
    return m * dt, np.array(out)


def fit_slope(tau, sd, slope, at):
    """Fit a line of fixed log-log slope where the local slope is closest to
    it, and read it at tau = at. Returns (value, local slope used, ok)."""
    lt, ls = np.log10(tau), np.log10(sd)
    local = np.gradient(ls, lt)
    i = int(np.argmin(np.abs(local - slope)))
    ok = abs(local[i] - slope) < 0.15
    b = ls[i] - slope * lt[i]
    return 10 ** (slope * np.log10(at) + b), local[i], ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--skip", type=float, default=2.0, help="minutes cut from the start (warm-up)")
    ap.add_argument("--stop", type=float, default=None, help="minute to stop at")
    ap.add_argument("--png", default=None)
    a = ap.parse_args()

    fast, slow, bad, _ = decode(open(a.log, "rb").read())
    if len(fast) == 0:
        sys.exit("no frames in that file")

    c = fast[:, 0] - fast[0, 0]
    lost = int(c[-1] + 1 - len(c))
    print(f"{len(fast)} frames, {len(fast) / FRAME_HZ / 60:.1f} min, "
          f"{lost} lost ({100 * lost / (c[-1] + 1):.3f}%), {bad} bad checksums")
    if len(slow):
        print(f"console bytes dropped by the board: {int(slow[-1, 3])}")

    t = c / FRAME_HZ / 60.0
    keep = t >= a.skip
    if a.stop is not None:
        keep &= t <= a.stop
    fast, c = fast[keep], c[keep]

    # Lost frames are filled by interpolation: the Allan sum assumes an even
    # grid, and a handful of interpolated points in an hour moves nothing.
    grid = np.arange(c[0], c[-1] + 1)
    data = np.column_stack([np.interp(grid, c, fast[:, k]) for k in range(1, 7)])

    names = ["accel x", "accel y", "accel z", "gyro x", "gyro y", "gyro z"]
    mean = data.mean(axis=0)
    print(f"\nmean accel  [{mean[0]:+.4f} {mean[1]:+.4f} {mean[2]:+.4f}] m/s^2  "
          f"|a| = {np.linalg.norm(mean[:3]):.4f}")
    print(f"mean gyro   [{mean[3]:+.6f} {mean[4]:+.6f} {mean[5]:+.6f}] rad/s "
          f"(this IS the bias the filter must estimate)")
    g = mean[:3] / np.linalg.norm(mean[:3])
    print(f"gravity direction in the IMU frame [{g[0]:+.3f} {g[1]:+.3f} {g[2]:+.3f}] "
          f"- expected [0 0 +1] if the chip is mounted as the URDF says and the robot is level")

    res = {}
    curves = {}
    print(f"\n{'axis':8s} {'N (density)':>14s} {'B (bias inst.)':>15s} {'K (bias walk)':>15s}")
    for k, nm in enumerate(names):
        tau, sd = adev(data[:, k], FRAME_HZ)
        curves[nm] = (tau, sd)
        N, _, okN = fit_slope(tau, sd, -0.5, 1.0)
        B = sd.min() / 0.664
        K, _, okK = fit_slope(tau, sd, +0.5, 3.0)
        if not okK:
            # No +1/2 region: the last point bounds it from above.
            K = sd[-1] * np.sqrt(3.0 / tau[-1])
        res[nm] = (N, B, K, okN, okK)
        print(f"{nm:8s} {N:14.3e} {B:15.3e} {K:15.3e}"
              f"{'' if okN else '  (no clean -1/2 region)'}"
              f"{'' if okK else '  (K: upper bound, no +1/2 region)'}")

    def worst(prefix, i):
        return max(res[f"{prefix} {ax}"][i] for ax in "xyz")

    print("\nsuggested inekf_params_t (worst axis, x1.5 margin):")
    print(f"  noise_gyro       = {1.5 * worst('gyro', 0):.2e}f   /* rad/s/sqrt(Hz)  */")
    print(f"  noise_accel      = {1.5 * worst('accel', 0):.2e}f   /* m/s^2/sqrt(Hz)  */")
    print(f"  noise_gyro_bias  = {1.5 * worst('gyro', 2):.2e}f   /* rad/s^2/sqrt(Hz) */")
    print(f"  noise_accel_bias = {1.5 * worst('accel', 2):.2e}f   /* m/s^3/sqrt(Hz)  */")
    print("  current defaults: 2.0e-3, 4.0e-2, 1.0e-3, 1.0e-3")

    if a.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(12, 5))
        for k, nm in enumerate(names):
            tau, sd = curves[nm]
            ax[k // 3].loglog(tau, sd, label=nm)
        ax[0].set_title("accelerometer"); ax[0].set_ylabel("Allan deviation, m/s^2")
        ax[1].set_title("gyro (uncalibrated)"); ax[1].set_ylabel("Allan deviation, rad/s")
        for x in ax:
            x.set_xlabel("tau, s"); x.grid(True, which="both", alpha=0.3); x.legend()
        fig.tight_layout()
        fig.savefig(a.png, dpi=120)
        print(f"\nplot: {a.png}")


if __name__ == "__main__":
    main()
