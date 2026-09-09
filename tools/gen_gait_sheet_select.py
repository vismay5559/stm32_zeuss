#!/usr/bin/env python3
"""
Turn the reference gait spreadsheet into a C table.

    python tools/gen_gait.py reference_gait_rleg_40ms_250hz.xlsx --sheet 0.4

Writes Appli/App/gait_ref.c and .h. Re-run it whenever the trajectory changes
rather than editing the generated table - hand-editing 200 rows of floats is
how a sign error gets in and stays in.

The spreadsheet is in DEGREES at the joint output. ODrive Set_Input_Pos wants
TURNS, and each drive is already configured with its own gear ratio, so the
only conversion needed is degrees -> turns on the output shaft: /360.
"""

import re
import sys
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Spreadsheet column -> joint slot. The leg test drives joints in node order,
# which is NOT the spreadsheet's column order.
#
#   node 1 = hip_roll    node 2 = hip_pitch    node 3 = knee    node 4 = ankle
#
# so slot 0 must be fed from the hipRoll column, slot 1 from hipP, and so on.
COLUMN_FOR_SLOT = [2, 1, 3, 4]          # xlsx column index per joint slot
SLOT_NAME = ["hip_roll", "hip_pitch", "knee", "ankle"]

DEG_TO_TURNS = 1.0 / 360.0


def read_sheet(z, name):
    x = z.read(name).decode()
    rows = []
    for r in re.findall(r"<row[^>]*>(.*?)</row>", x, re.S):
        cells = re.findall(
            r'<c r="[A-Z]+\d+"[^>]*>(?:<v>([^<]*)</v>|<is><t>([^<]*)</t></is>)?', r)
        rows.append([(v or inl) for v, inl in cells])
    return rows


def workbook_sheets(z):
    """Return {Excel sheet name: worksheet XML path}."""
    ns = {
        "main": "http://schemas.openxmlformats.org/spreadsheetml/2006/main",
        "rel": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
        "pkgrel": "http://schemas.openxmlformats.org/package/2006/relationships",
    }

    wb = ET.fromstring(z.read("xl/workbook.xml"))
    rels = ET.fromstring(z.read("xl/_rels/workbook.xml.rels"))

    rid_to_target = {
        r.attrib["Id"]: r.attrib["Target"]
        for r in rels.findall("pkgrel:Relationship", ns)
    }

    result = {}
    for sheet in wb.find("main:sheets", ns):
        name = sheet.attrib["name"]
        rid = sheet.attrib["{" + ns["rel"] + "}id"]
        target = rid_to_target[rid]
        path = target.lstrip("/")
        if not path.startswith("xl/"):
            path = "xl/" + path
        result[name] = path
    return result


def parse_args():
    src = Path(sys.argv[1] if len(sys.argv) > 1
               and not sys.argv[1].startswith("-")
               else ROOT / "reference_gait_rleg_40ms_250hz.xlsx")
    sheet = None

    i = 2 if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 1
    while i < len(sys.argv):
        if sys.argv[i] in ("--sheet", "-s"):
            if i + 1 >= len(sys.argv):
                raise SystemExit("--sheet requires a sheet name")
            sheet = sys.argv[i + 1]
            i += 2
        else:
            raise SystemExit(f"unknown argument: {sys.argv[i]}")
    return src, sheet


def main():
    src, requested_sheet = parse_args()
    z = zipfile.ZipFile(src)

    sheets = workbook_sheets(z)
    print("Available sheets: " + ", ".join(sheets))

    if requested_sheet is None:
        sheet_name = next(iter(sheets))
    else:
        if requested_sheet not in sheets:
            raise SystemExit(
                f"Sheet '{requested_sheet}' not found. "
                f"Use one of: {', '.join(sheets)}"
            )
        sheet_name = requested_sheet

    ang = read_sheet(z, sheets[sheet_name])

    # Optional metadata sheet; no longer assumes sheet2.xml exists.
    info = {}
    for candidate in ("info", "Info", "metadata", "Metadata"):
        if candidate in sheets:
            rows = read_sheet(z, sheets[candidate])
            info = {r[0]: r[1] for r in rows[1:] if len(r) > 1}
            break

    print(f"  parsing sheet: {sheet_name}")

    header, data = ang[0], ang[1:]
    n = len(data)
    dt = float(data[1][0]) - float(data[0][0])
    cycle = n * dt

    print(f"{src.name}: {n} samples, dt {dt:.4f} s, cycle {cycle:.3f} s "
          f"({1/dt:.0f} Hz)")
    print(f"  source: {info.get('source', '?')}")
    print(f"  leg {info.get('leg', '?')}, {info.get('speed_m_s', '?')} m/s")
    print()

    # degrees -> turns, reordered into joint-slot order
    table = []
    for row in data:
        table.append([float(row[COLUMN_FOR_SLOT[s]]) * DEG_TO_TURNS
                      for s in range(4)])

    print("  slot        column           degrees            turns")
    print("  " + "-" * 62)
    for s in range(4):
        col = header[COLUMN_FOR_SLOT[s]]
        degs = [float(r[COLUMN_FOR_SLOT[s]]) for r in data]
        trns = [t[s] for t in table]
        print(f"  {SLOT_NAME[s]:<11} {col:<14} "
              f"{min(degs):+7.2f}..{max(degs):+7.2f}   "
              f"{min(trns):+7.4f}..{max(trns):+7.4f}")

    # ------------------------------------------------------------------
    # 1. Close the interior discontinuities.
    #
    # This trajectory is two half-strides concatenated, and they do not meet.
    # Every joint jumps at once at t = cycle/2 - sample 99 -> 100 - by 1.7 to
    # 2.0 degrees where the neighbouring steps are 0.03. A leg cannot move that
    # far in one 4 ms sample, so the drives see a step input once per half
    # cycle and ring afterwards. On the knee, whose whole travel is 3.7
    # degrees, that step is 44% of the gait: there is no gain setting that
    # follows it, and it makes the joint impossible to tune.
    #
    # Tilting the whole path (step 2 below) cannot help here - that only moves
    # the ends. This closes the gap locally instead: each side is pulled
    # halfway towards the other, tapered to nothing over BLEND_SAMPLES either
    # side with a smoothstep, so velocity stays continuous at the edges of the
    # window and the rest of the trajectory is untouched.
    #
    # The window is a compromise. Too narrow and the correction itself becomes
    # a fast move; too wide and it distorts real motion. 20 samples at 250 Hz
    # is 80 ms, over which 2 degrees is 25 deg/s - inside the gait's own peak
    # of about 32 deg/s, so nothing here asks for a speed the trajectory does
    # not already contain.
    #
    # This is a REPAIR, not a fix. The trajectory optimisation should produce a
    # continuous stride; if it can be re-run with the two halves constrained to
    # meet, do that instead and this becomes a no-op.
    # ------------------------------------------------------------------
    BLEND_SAMPLES = 20
    STEP_THRESHOLD = 0.05          # of the joint's span

    def smoothstep(t):
        return t * t * (3.0 - 2.0 * t)

    # Find the JOIN INDICES first, across every joint at once.
    #
    # The discontinuity is a property of the trajectory, not of any one joint:
    # all four jump at the same sample because that is where two half-strides
    # were concatenated. Judging each joint alone gets this wrong - the ankle's
    # jump is only 5.5x its own median step and would be dismissed as ordinary
    # motion, while the knee's is 49x. Once two or more joints jump together at
    # the same index, that index is a stitch and every joint is repaired there,
    # however mild its own share looks.
    medians = []
    for s_i in range(4):
        col = [row[s_i] for row in table]
        steps = sorted(abs(col[k + 1] - col[k]) for k in range(n - 1))
        medians.append(max(steps[len(steps) // 2], 1e-9))

    spans = []
    for s_i in range(4):
        col = [row[s_i] for row in table]
        spans.append(max(max(col) - min(col), 1e-9))

    joins = []
    for k in range(n - 1):                      # interior only; wrap is step 2
        votes = 0
        alone = False

        for s_i in range(4):
            step = abs(table[k + 1][s_i] - table[k][s_i])

            if step > medians[s_i] * 4.0:
                votes += 1

            # A single joint is enough on its own if its step is also large
            # against its OWN travel. Two joints agreeing is good evidence of a
            # stitch, but it is not the only evidence: this trajectory steps
            # only hip_pitch at the half-cycle, by 7% of its span, and the
            # two-vote rule let it through - so the repair reported nothing to
            # do and the check immediately flagged the step it should have
            # removed.
            if (step > spans[s_i] * STEP_THRESHOLD) and (step > medians[s_i] * 4.0):
                alone = True

        if (votes >= 2) or alone:
            joins.append(k)

    repairs = []

    for k in joins:
        for s_i in range(4):
            col = [row[s_i] for row in table]
            span = max(col) - min(col)
            jump = col[k + 1] - col[k]
            typical = medians[s_i]

            excess = jump - (typical if jump > 0 else -typical)
            half = excess / 2.0

            # Pull the earlier side forward and the later side back, so the
            # join closes without shifting either half of the trajectory as a
            # whole.
            for w in range(BLEND_SAMPLES):
                amount = half * smoothstep((BLEND_SAMPLES - w) / float(BLEND_SAMPLES))

                lo = k - w
                if lo >= 0:
                    table[lo][s_i] += amount

                hi = k + 1 + w
                if hi < n:
                    table[hi][s_i] -= amount

            repairs.append((SLOT_NAME[s_i], k, excess, span))

    if repairs:
        print("\n  interior steps repaired (blended over "
              f"{BLEND_SAMPLES} samples either side):")
        for name, k, excess, span in repairs:
            print(f"    {name:<11} {k}->{k+1}  step {excess:+.4f} turns "
                  f"({abs(excess) * 360:.2f} deg, {abs(excess) / span * 100:.0f}% "
                  f"of span)")
    else:
        print("\n  no interior steps above threshold")

    # ------------------------------------------------------------------
    # 2. Close the wrap seam, 199 -> 0.
    #
    # A different problem needing a different fix. The last sample not meeting
    # the first is a constant offset over a whole cycle, so it can be absorbed
    # by tilting the entire path - spreading it as a tiny slope rather than
    # leaving it as one kick per stride. Doing this locally, as above, would
    # put all of it in a few samples for no reason.
    # ------------------------------------------------------------------
    for s_i in range(4):
        offset = table[0][s_i] - table[-1][s_i]
        for i in range(n):
            table[i][s_i] += offset * (i / float(n - 1))

    # ------------------------------------------------------------------
    # 3. Report what is left, EVERY large step and not just the worst.
    #
    # The check this replaces compared against a fixed 0.01 turns. That is 3.6
    # degrees - most of the knee's entire travel - so a step could be nearly
    # half that joint's motion and still pass silently, which is exactly what
    # happened. A threshold has to be relative to the joint it is judging.
    # ------------------------------------------------------------------
    print("\n  step check (threshold: "
          f"{STEP_THRESHOLD * 100:.0f}% of each joint's span):")
    worst_any = False

    for s_i in range(4):
        col = [row[s_i] for row in table]
        span = max(col) - min(col)
        if span < 1e-6:
            print(f"    {SLOT_NAME[s_i]:<11} span is zero - joint does not move")
            continue

        big = []
        for k in range(n):
            m = (k + 1) % n
            step = abs(col[m] - col[k])
            if step > span * STEP_THRESHOLD:
                big.append((step, k, m))

        worst = max(abs(col[(k + 1) % n] - col[k]) for k in range(n))
        pct = worst / span * 100.0

        if big:
            worst_any = True
            print(f"    {SLOT_NAME[s_i]:<11} span {span:.4f}   "
                  f"worst step {worst:.4f} ({pct:.0f}% of span)  "
                  f"<-- {len(big)} STEP(S) OVER THRESHOLD")
            for step, k, m in sorted(big, reverse=True)[:5]:
                print(f"                  {k}->{m}  {step:.4f} turns "
                      f"({step * 360:.2f} deg, {step / span * 100:.0f}%)")
        else:
            print(f"    {SLOT_NAME[s_i]:<11} span {span:.4f}   "
                  f"worst step {worst:.4f} ({pct:.0f}% of span)")

    if worst_any:
        print("\n  A step above threshold is a kick the drives cannot follow."
              "\n  Tune nothing until it is gone - the joint will be ringing,"
              "\n  not tracking.")

    hdr = ROOT / "Appli" / "App" / "gait_ref.h"
    src_c = ROOT / "Appli" / "App" / "gait_ref.c"

    hdr.write_text(f'''#ifndef GAIT_REF_H
#define GAIT_REF_H

#include <stdint.h>

/*
 * Reference gait for the right leg, generated by tools/gen_gait.py from
 * {src.name}.
 *
 * DO NOT EDIT THIS TABLE BY HAND - regenerate it. Source was
 * {info.get('source', 'unknown')}.
 *
 * Values are ODrive Set_Input_Pos TURNS on the OUTPUT shaft. The spreadsheet
 * is in degrees; each drive is already configured with its gear ratio, so the
 * conversion is simply /360.
 *
 * Column order is JOINT SLOT order, which follows the CAN node IDs and is not
 * the spreadsheet's column order:
 *
 *   slot 0 = hip_roll  (node 1)      slot 2 = knee   (node 3)
 *   slot 1 = hip_pitch (node 2)      slot 3 = ankle  (node 4)
 */

/* Column index per joint, for picking one out when only some drives exist. */
#define GAIT_COL_HIP_ROLL   0
#define GAIT_COL_HIP_PITCH  1
#define GAIT_COL_KNEE       2
#define GAIT_COL_ANKLE      3

#define GAIT_SAMPLES     {n}
#define GAIT_JOINTS      4
#define GAIT_DT_S        {dt:.6f}f
#define GAIT_RATE_HZ     {1/dt:.1f}f
#define GAIT_CYCLE_S     {cycle:.6f}f

extern const float g_gait_turns[GAIT_SAMPLES][GAIT_JOINTS];

/*
 * Where every joint should be at one moment in a walking stride.
 *
 * `phase` is how far through a single stride the robot is: 0.0 at the start,
 * 1.0 at the end, and back to 0.0 to begin the next one. Feed it a steadily
 * rising phase and it walks out one full step cycle. Values outside 0..1 wrap,
 * so a free-running phase clock needs no special case at the seam.
 *
 * `turns_out` receives one target per joint, in turns on the OUTPUT shaft.
 *
 * Linearly interpolates between the {1/dt:.0f} Hz samples, so a 1 kHz caller
 * gets a smooth ramp rather than a staircase.
 */
void gait_sample(float phase, float *turns_out);

/*
 * As gait_sample, but also returns the trajectory's VELOCITY in turns/s.
 *
 * The velocity is at NOMINAL playback - one cycle per GAIT_CYCLE_S. A caller
 * playing the gait at a different rate must scale it by the same factor it
 * scales the phase clock, or the feedforward will not match the motion it is
 * feeding forward. Playing at 0.25x with unscaled velocity asks the drive to
 * move four times faster than the position setpoint is going.
 *
 * Central difference over the neighbouring table samples, then interpolated,
 * so the result is continuous rather than the piecewise-constant staircase a
 * plain forward difference of the interpolated position would give.
 *
 * turns_out or vel_out may be NULL if only one is wanted.
 */
void gait_sample_vel(float phase, float *turns_out, float *vel_out);

#endif /* GAIT_REF_H */
''')

    rows = "\n".join(
        "    { " + ", ".join(f"{v:+.6f}f" for v in row) + " },"
        for row in table)

    src_c.write_text(f'''#include "gait_ref.h"

/* Generated by tools/gen_gait.py from {src.name}. Do not edit. */
const float g_gait_turns[GAIT_SAMPLES][GAIT_JOINTS] = {{
{rows}
}};

void gait_sample(float phase, float *turns_out)
{{
    /* Wrap into [0,1). The caller's phase clock is free-running, so this is
       what makes the trajectory periodic. */
    phase -= (float)(int)phase;
    if (phase < 0.0f)
    {{
        phase += 1.0f;
    }}

    float    x = phase * (float)GAIT_SAMPLES;
    int      i = (int)x;
    float    f = x - (float)i;
    int      j = (i + 1) % GAIT_SAMPLES;   /* wraps at the seam */

    if (i >= GAIT_SAMPLES)
    {{
        i = GAIT_SAMPLES - 1;
        j = 0;
        f = 0.0f;
    }}

    for (int k = 0; k < GAIT_JOINTS; k++)
    {{
        turns_out[k] = g_gait_turns[i][k] +
                       (g_gait_turns[j][k] - g_gait_turns[i][k]) * f;
    }}
}}

/*
 * Velocity of the trajectory at table row i, in turns/s at nominal playback.
 *
 * Central difference across the neighbours. The gait is a closed cycle, so the
 * indices wrap rather than clamp - a one-sided difference at the seam would
 * put a false velocity step exactly where the cycle repeats, which is the one
 * place a discontinuity would be felt on every single lap.
 */
static float row_vel(int i, int k)
{{
    int prev = (i - 1 + GAIT_SAMPLES) % GAIT_SAMPLES;
    int next = (i + 1) % GAIT_SAMPLES;

    return (g_gait_turns[next][k] - g_gait_turns[prev][k]) /
           (2.0f * GAIT_DT_S);
}}

void gait_sample_vel(float phase, float *turns_out, float *vel_out)
{{
    phase -= (float)(int)phase;
    if (phase < 0.0f)
    {{
        phase += 1.0f;
    }}

    float    x = phase * (float)GAIT_SAMPLES;
    int      i = (int)x;
    float    f = x - (float)i;
    int      j = (i + 1) % GAIT_SAMPLES;

    if (i >= GAIT_SAMPLES)
    {{
        i = GAIT_SAMPLES - 1;
        j = 0;
        f = 0.0f;
    }}

    for (int k = 0; k < GAIT_JOINTS; k++)
    {{
        /* Either output may be NULL - a caller that only wants position calls
           gait_sample(), but one that only wants velocity has nowhere else to
           go, and writing through a null pointer to save a branch is not a
           trade worth making in a 1 kHz loop. */
        if (turns_out != 0)
        {{
            turns_out[k] = g_gait_turns[i][k] +
                           (g_gait_turns[j][k] - g_gait_turns[i][k]) * f;
        }}

        if (vel_out != 0)
        {{
            float vi = row_vel(i, k);
            float vj = row_vel(j, k);

            vel_out[k] = vi + (vj - vi) * f;
        }}
    }}
}}
''')

    print(f"\nwrote {hdr.relative_to(ROOT)} and {src_c.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())