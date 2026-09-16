#!/usr/bin/env bash
#
# The estimator against a simulated walk with exact ground truth.
#
#   gen_walk.py   zeus.urdf -> a walk, and what the sensors would have said
#   replay        the REAL fusion.c / inekf.c / zeus_kinematics.c on that data
#   evaluate.py   estimate vs truth: tilt, body velocity, height, foot height
#
#     tools/sim/run.sh                         # prints the errors
#     tools/sim/run.sh --plot                  # and writes out/estimate.png
#     tools/sim/run.sh --check                 # exit 1 beyond the limits (CI)
#     tools/sim/run.sh -- --steps 30 --seed 7  # anything after -- goes to gen_walk.py
#
# Environment:
#   PY        python with Pinocchio, for gen_walk.py   (default python3)
#   EVAL_PY   python with numpy (+ matplotlib for --plot) (default $PY)
#   URDF      the model                    (default ../zeus_26/.../zeus.urdf)
#   OUT       where everything goes        (default tools/sim/out)
#   CC        C compiler                   (default cc)
#
# On a ROS machine, point PY at a venv and unset PYTHONPATH - ROS's eigenpy
# breaks Pinocchio's:
#   PY=~/kin_venv/bin/python EVAL_PY=python3 env -u PYTHONPATH tools/sim/run.sh --plot

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
APP="$ROOT/Appli/App"
PY="${PY:-python3}"
EVAL_PY="${EVAL_PY:-$PY}"
OUT="${OUT:-$HERE/out}"
CC="${CC:-cc}"
URDF="${URDF:-$(dirname "$ROOT")/zeus_26/zeus_description/urdf/zeus.urdf}"

eval_args=()
gen_args=()
while [ $# -gt 0 ]; do
    case "$1" in
        --) shift; gen_args=("$@"); break ;;
        *)  eval_args+=("$1"); shift ;;
    esac
done

mkdir -p "$OUT"

echo "=== building replay (the firmware estimator, for the host) ==="
$CC -std=c11 -O2 -Wall -Wextra -Wconversion -Wshadow -Werror -DNEXUS_HOSTTEST=1 \
    -I"$ROOT/tools/hosttest/stub" -I"$APP" \
    -o "$OUT/replay" \
    "$HERE/replay.c" "$APP/fusion.c" "$APP/inekf.c" "$APP/lie_group.c" \
    "$APP/zeus_kinematics.c" "$APP/robot_config.c" -lm

echo "=== simulating a walk from $URDF ==="
"$PY" "$HERE/gen_walk.py" --urdf "$URDF" --out "$OUT" "${gen_args[@]}"

echo "=== replaying it through fusion.c ==="
"$OUT/replay" "$OUT/input.bin" "$OUT/estimate.bin"

echo "=== estimate vs truth ==="
"$EVAL_PY" "$HERE/evaluate.py" "$OUT" "${eval_args[@]}"
