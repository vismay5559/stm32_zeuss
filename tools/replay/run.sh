#!/usr/bin/env bash
# Replay a recording through the firmware estimator: run.sh RECORDING [every_n_ticks] [extra CFLAGS]
set -eu
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP="$HERE/../../Appli/App"; STUB="$HERE/../hosttest/stub"; OUT="$HERE/build"
mkdir -p "$OUT"
cc -std=c11 -O2 -Wall -DNEXUS_HOSTTEST=1 ${3:-} -I"$STUB" -I"$APP" -o "$OUT/replay_fusion" \
   "$HERE/replay_fusion.c" "$APP/fusion.c" "$APP/inekf.c" "$APP/lie_group.c" \
   "$APP/zeus_kinematics.c" "$APP/robot_config.c" -lm
python3 "$HERE/ticks_from_recording.py" "$1" "$OUT/rec.ticks"
"$OUT/replay_fusion" "$OUT/rec.ticks" "${2:-1000}"
