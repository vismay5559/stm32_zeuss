#!/usr/bin/env python3
"""
Pull the trajectory capture out of a PuTTY log, plot it, keep it for comparison.

    python tools/plot_run.py                          plot the newest run
    python tools/plot_run.py -l vg1.0_vi5.0           ...and archive it under that name
    python tools/plot_run.py --compare                overlay everything kept
    python tools/plot_run.py other.log                read a different log

With no filename it reads DEFAULT_LOG below, which is where PuTTY is pointed.
Tuning means running this twenty times in an afternoon; having to type the path
every time is twenty chances to plot the wrong file.

Tuning is a loop - change a gain, run the leg, look, change it again - and that
loop only closes if you can see the previous attempt beside this one. A single
plot tells you the run was bad; two plots tell you whether the change helped.
So every labelled run is written to runs/ and --compare puts them on one axis.

Label runs with the gains that produced them. `-l vg0.6_vi3.0` is worth the
four seconds of typing; `-l try4` will be meaningless by tomorrow.

The board prints a capture between a `t_s,cmd_turns,...` header and an
`===== end =====` marker, buried in however many thousand lines of CAN frame
trace. This digs it out whatever else got interleaved with it.
"""

import argparse
import sys
from datetime import datetime
from pathlib import Path

HEADER = "t_s,cmd_turns,pos_turns,vel_tps,trq_Nm"
END = "===== end ====="

RUNS_DIR = Path(__file__).resolve().parent.parent / "runs"

# Where PuTTY is logging. Change this if you move the log; everything else
# follows. PuTTY must be set to "All session output" with "Always append to the
# end of it" - on append the newest capture is the one just flown, which is
# what this reads. On overwrite you would silently lose every earlier run.
DEFAULT_LOG = Path(r"C:\Users\visma\Downloads\putty.log")

# A joint is "stuck" when the command is clearly moving and the output clearly
# is not. The gait peaks near 0.18 turn/s at 0.25x speed, so 0.02 turn/s means
# the command is genuinely going somewhere; 0.005 turn/s (1.8 deg/s) is the
# encoder sitting still. The gap between the two thresholds is what stops
# ordinary turnaround points at the ends of the stroke counting as sticking.
CMD_MOVING_TPS = 0.02
POS_STILL_TPS = 0.005


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


def metrics(rows):
    """Numbers that say what to change next, not just how bad it was."""
    t = [r[0] for r in rows]
    cmd = [r[1] for r in rows]
    pos = [r[2] for r in rows]
    vel = [r[3] for r in rows]
    trq = [r[4] for r in rows]
    n = len(rows)

    err = [c - p for c, p in zip(cmd, pos)]
    rms = (sum(e * e for e in err) / n) ** .5

    dt = (t[-1] - t[0]) / (n - 1) if n > 1 else 0.01

    # Peak-to-peak ratio flatters a joint that lurches past the setpoint and
    # back - a leg thrashing at 300% of the commanded stroke scores better than
    # one that lags slightly. It is reported because it is easy to recognise
    # from the old logs, not because it is worth optimising.
    cmd_pp = max(cmd) - min(cmd)
    pos_pp = max(pos) - min(pos)

    # Stiction, measured directly - but split by where it happens, because the
    # two kinds mean different things and get fixed differently.
    #
    # Mid-stroke sticking is the joint refusing to move while the command
    # sweeps past it. That is missing torque authority and it is what raising
    # vel_gain cures. Drive it to zero first.
    #
    # Turnaround sticking is the joint pausing at a direction reversal, where
    # the command itself is barely moving and the joint has to come to rest and
    # start again. Some of that is unavoidable in any geared joint. Lumping the
    # two together hides the moment when the real problem is solved: mid-stroke
    # can hit zero while the total barely moves.
    cmd_vels = [abs(cmd[i] - cmd[i - 1]) / dt for i in range(1, n)]
    fast = max(cmd_vels) * 0.35 if cmd_vels else 0

    stuck = stuck_mid = 0
    for i in range(1, n):
        if cmd_vels[i - 1] > CMD_MOVING_TPS and abs(vel[i]) < POS_STILL_TPS:
            stuck += 1
            if cmd_vels[i - 1] > fast:
                stuck_mid += 1
    stuck_pct = stuck / (n - 1) * 100 if n > 1 else 0
    stuck_mid_pct = stuck_mid / (n - 1) * 100 if n > 1 else 0

    # How far past the command the joint travels at the ends of the stroke.
    # Once stiction is beaten this becomes the dominant error, and it points at
    # the integrator rather than at vel_gain: the joint is still being pushed
    # when the command has already turned around.
    overshoot = max((min(cmd) - min(pos)), (max(pos) - max(cmd)), 0.0) * 360

    # How far behind the measured trace is, found by sliding it against the
    # command and taking the shift that fits best. Honest lag is a gain
    # problem; a stuck joint produces a meaningless number here, which is why
    # it is only printed when the joint is actually moving.
    lag_ms = None
    if stuck_mid_pct < 5:
        best, best_err = 0, None
        for shift in range(0, min(60, n // 4)):
            e = sum((cmd[i] - pos[i + shift]) ** 2
                    for i in range(n - shift)) / (n - shift)
            if best_err is None or e < best_err:
                best, best_err = shift, e
        lag_ms = best * dt * 1000

    return {
        "n": n, "dur": t[-1] - t[0], "dt": dt,
        "cmd_pp_deg": cmd_pp * 360, "pos_pp_deg": pos_pp * 360,
        "track_pct": pos_pp / cmd_pp * 100 if cmd_pp > 1e-6 else 0,
        "rms_deg": rms * 360,
        "worst_deg": max(abs(e) for e in err) * 360,
        "worst_at": t[max(range(n), key=lambda i: abs(err[i]))],
        "stuck_pct": stuck_pct,
        "stuck_mid_pct": stuck_mid_pct,
        "overshoot_deg": overshoot,
        "lag_ms": lag_ms,
        "peak_trq": max(abs(x) for x in trq),
    }


def report(m, indent="  "):
    print(f"{indent}{m['n']} samples over {m['dur']:.2f} s")
    print(f"{indent}commanded travel : {m['cmd_pp_deg']:7.2f} deg")
    print(f"{indent}achieved  travel : {m['pos_pp_deg']:7.2f} deg  "
          f"({m['track_pct']:.0f}% - ignore, see comment)")
    print(f"{indent}RMS error        : {m['rms_deg']:7.2f} deg   <- tune on this")
    print(f"{indent}worst error      : {m['worst_deg']:7.2f} deg  "
          f"at t={m['worst_at']:.2f}s")
    print(f"{indent}stuck mid-stroke : {m['stuck_mid_pct']:7.1f} %     <- and this")
    print(f"{indent}stuck at reversal: {m['stuck_pct'] - m['stuck_mid_pct']:7.1f} %"
          f"     (some is unavoidable)")
    print(f"{indent}overshoot        : {m['overshoot_deg']:7.2f} deg  "
          f"past the end of the stroke")
    if m["lag_ms"] is None:
        print(f"{indent}lag              :       -       "
              f"(sticking mid-stroke, lag not meaningful)")
    else:
        print(f"{indent}lag              : {m['lag_ms']:7.0f} ms")
    print(f"{indent}peak torque      : {m['peak_trq']:7.3f} Nm")

    # One instruction, in the order the problems have to be fixed. Chasing
    # overshoot while the joint is still seizing mid-stroke just moves the
    # error around.
    if m["stuck_mid_pct"] > 5:
        print(f"{indent}--> seizing mid-stroke {m['stuck_mid_pct']:.0f}% of the time. "
              f"Raise vel_gain.")
    elif m["overshoot_deg"] > 1.5:
        print(f"{indent}--> mid-stroke is clean, but it sails {m['overshoot_deg']:.1f} deg "
              f"past each end.\n{indent}    Raise vel_gain further, or cut "
              f"vel_integrator_gain to 2x vel_gain.")
    elif m["lag_ms"] is not None and m["lag_ms"] > 80:
        print(f"{indent}--> moving but {m['lag_ms']:.0f} ms behind. Raise pos_gain.")
    else:
        print(f"{indent}--> tracking cleanly. Save it: odrv0.save_configuration()")


def write_csv(rows, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(HEADER + "\n" +
                    "\n".join(",".join(f"{v:g}" for v in r) for r in rows) + "\n")
    return path


def read_csv(path):
    rows = []
    for line in path.read_text().splitlines()[1:]:
        parts = line.split(",")
        if len(parts) == 5:
            try:
                rows.append(tuple(float(p) for p in parts))
            except ValueError:
                pass
    return rows


def plot_single(rows, title, png):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("pip install matplotlib to get a plot")
        return

    t = [r[0] for r in rows]
    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    ax[0].plot(t, [r[1] * 360 for r in rows], label="commanded", lw=2)
    ax[0].plot(t, [r[2] * 360 for r in rows], label="measured", lw=1.4)
    ax[0].set_ylabel("degrees")
    ax[0].legend(loc="upper right")
    ax[0].grid(alpha=.3)
    ax[0].set_title(title)

    err = [(r[1] - r[2]) * 360 for r in rows]
    ax[1].plot(t, err, color="crimson")
    ax[1].axhline(0, color="k", lw=.6)
    ax[1].set_ylabel("error, deg")
    ax[1].grid(alpha=.3)

    # Shade where the joint is stuck. Flat red bands are stiction; if they
    # vanish as vel_gain goes up, that is the gain doing its job.
    dt = (t[-1] - t[0]) / (len(t) - 1) if len(t) > 1 else .01
    for i in range(1, len(rows)):
        cmd_vel = abs(rows[i][1] - rows[i - 1][1]) / dt
        if cmd_vel > CMD_MOVING_TPS and abs(rows[i][3]) < POS_STILL_TPS:
            ax[1].axvspan(t[i - 1], t[i], color="red", alpha=.10, lw=0)

    ax[2].plot(t, [r[4] for r in rows], color="darkorange")
    ax[2].set_ylabel("torque, Nm")
    ax[2].set_xlabel("seconds")
    ax[2].grid(alpha=.3)

    fig.tight_layout()
    fig.savefig(png, dpi=130)
    print(f"wrote {png}")
    plt.show()


def plot_compare(runs):
    """Every archived run's error trace on one axis - the tuning view."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("pip install matplotlib to get a plot")
        return

    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    for name, rows in runs:
        t = [r[0] for r in rows]
        ax[0].plot(t, [(r[1] - r[2]) * 360 for r in rows], lw=1.3, label=name)
        ax[1].plot(t, [r[4] for r in rows], lw=1.1, label=name)

    ax[0].axhline(0, color="k", lw=.6)
    ax[0].set_ylabel("error, deg")
    ax[0].set_title("tracking error by run - flatter is better")
    ax[0].legend(fontsize=8, loc="upper right")
    ax[0].grid(alpha=.3)

    ax[1].set_ylabel("torque, Nm")
    ax[1].set_xlabel("seconds")
    ax[1].grid(alpha=.3)

    fig.tight_layout()
    png = RUNS_DIR / "compare.png"
    fig.savefig(png, dpi=130)
    print(f"wrote {png}")
    plt.show()


def do_compare():
    paths = sorted(RUNS_DIR.glob("*.csv"))
    if not paths:
        print(f"nothing archived in {RUNS_DIR}")
        print("run with -l LABEL to keep a run for comparison")
        return 1

    runs = []
    print(f"{len(paths)} archived run(s)\n")
    print(f"{'run':<26} {'RMS deg':>8} {'stuck mid':>10} {'over deg':>9} "
          f"{'worst deg':>10} {'lag ms':>7} {'peak Nm':>8}")
    print("-" * 82)

    for p in paths:
        rows = read_csv(p)
        if not rows:
            continue
        m = metrics(rows)
        runs.append((p.stem, rows))
        lag = "-" if m["lag_ms"] is None else f"{m['lag_ms']:.0f}"
        print(f"{p.stem:<26} {m['rms_deg']:8.2f} {m['stuck_mid_pct']:10.1f} "
              f"{m['overshoot_deg']:9.2f} {m['worst_deg']:10.2f} {lag:>7} "
              f"{m['peak_trq']:8.3f}")

    print()
    best = min(runs, key=lambda r: metrics(r[1])["rms_deg"])
    print(f"best RMS so far: {best[0]}")

    plot_compare(runs)
    return 0


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("log", nargs="?", help=f"log to read (default {DEFAULT_LOG})")
    ap.add_argument("-l", "--label", help="archive this run under runs/LABEL.csv")
    ap.add_argument("--compare", action="store_true",
                    help="overlay every archived run instead of reading a log")
    ap.add_argument("-h", "--help", action="store_true")
    args = ap.parse_args()

    if args.help:
        print(__doc__)
        return 0

    if args.compare:
        return do_compare()

    log = Path(args.log) if args.log else DEFAULT_LOG
    if not log.exists():
        print(f"no such log: {log}")
        print("PuTTY must be logging 'All session output' to that path, or pass "
              "a filename.")
        return 1
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
        report(metrics(rows))
        print()

    rows = runs[-1]                        # newest is the one just flown
    write_csv(rows, log.with_suffix(".csv"))
    print(f"wrote {log.with_suffix('.csv')}")

    title = log.name
    if args.label:
        kept = write_csv(rows, RUNS_DIR / f"{args.label}.csv")
        print(f"archived {kept}")
        title = args.label
    else:
        stamp = datetime.now().strftime("%H%M%S")
        print(f"not archived - pass -l LABEL (e.g. -l vg0.6_vi3.0_{stamp}) "
              f"to keep it for --compare")

    plot_single(rows, title, log.with_suffix(".png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
