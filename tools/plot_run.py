#!/usr/bin/env python3
"""
Pull the trajectory CSV out of a PuTTY log and plot it.

    python tools/plot_run.py putty.log

The board prints a capture between a `t_s,cmd_turns,...` header and an
`===== end =====` marker, surrounded by however many thousand lines of CAN
frame trace. This finds the block, writes it out as a clean .csv next to the
log, and plots commanded against measured if matplotlib is installed.

Multiple runs in one log are all extracted; the newest is plotted.
"""

import re
import sys
from pathlib import Path

HEADER = "t_s,cmd_turns,pos_turns,vel_tps,trq_Nm"
END = "===== end ====="


def extract(text):
    """Return a list of runs, each a list of (t, cmd, pos, vel, trq) tuples."""
    runs = []
    rows = None

    for line in text.splitlines():
        line = line.strip()

        if HEADER in line:
            rows = []                      # a capture starts here
            continue

        if rows is None:
            continue

        if END in line:
            if rows:
                runs.append(rows)
            rows = None
            continue

        # Data lines are five comma-separated numbers and nothing else. The
        # frame trace can interleave if the console buffer wrapped, so every
        # line is validated rather than assumed.
        parts = line.split(",")
        if len(parts) != 5:
            continue
        try:
            rows.append(tuple(float(p) for p in parts))
        except ValueError:
            continue

    if rows:                               # log ended mid-capture
        runs.append(rows)
    return runs


def summarise(rows):
    t = [r[0] for r in rows]
    cmd = [r[1] for r in rows]
    pos = [r[2] for r in rows]
    trq = [r[4] for r in rows]

    cmd_pp = max(cmd) - min(cmd)
    pos_pp = max(pos) - min(pos)
    err = [abs(c - p) for c, p in zip(cmd, pos)]

    print(f"  {len(rows)} samples over {t[-1]:.2f} s")
    print(f"  commanded travel : {cmd_pp * 360:7.2f} deg")
    print(f"  achieved  travel : {pos_pp * 360:7.2f} deg")
    print(f"  tracking         : {pos_pp / cmd_pp * 100 if cmd_pp > 1e-6 else 0:7.0f} %")
    print(f"  worst error      : {max(err) * 360:7.2f} deg")
    print(f"  peak torque      : {max(abs(x) for x in trq):7.3f} Nm")

    # Where the error is worst says what is clipping. An even lag points at
    # position gain; a lag that appears only on the fast parts points at a
    # velocity limit.
    worst = err.index(max(err))
    print(f"  worst at         : t = {t[worst]:.2f} s "
          f"({worst / len(rows) * 100:.0f}% through the run)")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    log = Path(sys.argv[1])
    runs = extract(log.read_text(errors="replace"))

    if not runs:
        print(f"no capture found in {log}")
        print("\nThe board prints one when the gait completes. If it is missing:")
        print("  - the gait never finished (check for 'cycle(s) complete')")
        print("  - PuTTY logging was started after the dump")
        print("  - LEGTEST_CAPTURE is 0")
        return 1

    print(f"{len(runs)} capture(s) in {log.name}\n")
    for i, rows in enumerate(runs):
        print(f"run {i + 1}:")
        summarise(rows)
        print()

    rows = runs[-1]
    out = log.with_suffix(".csv")
    out.write_text(HEADER + "\n" +
                   "\n".join(",".join(f"{v:g}" for v in r) for r in rows) + "\n")
    print(f"wrote {out}")

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("pip install matplotlib to get a plot")
        return 0

    t = [r[0] for r in rows]
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    ax[0].plot(t, [r[1] * 360 for r in rows], label="commanded", lw=2)
    ax[0].plot(t, [r[2] * 360 for r in rows], label="measured", lw=1.4)
    ax[0].set_ylabel("degrees")
    ax[0].legend()
    ax[0].grid(alpha=.3)
    ax[0].set_title(log.name)

    ax[1].plot(t, [(r[1] - r[2]) * 360 for r in rows], color="crimson")
    ax[1].axhline(0, color="k", lw=.6)
    ax[1].set_ylabel("error, deg")
    ax[1].grid(alpha=.3)

    ax[2].plot(t, [r[4] for r in rows], color="darkorange")
    ax[2].set_ylabel("torque, Nm")
    ax[2].set_xlabel("seconds")
    ax[2].grid(alpha=.3)

    fig.tight_layout()
    png = log.with_suffix(".png")
    fig.savefig(png, dpi=130)
    print(f"wrote {png}")
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
