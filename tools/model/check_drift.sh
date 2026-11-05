#!/usr/bin/env bash
#
# Has the workspace's URDF moved on while our vendored copy stayed put?
#
# This is the one model check CI cannot do. The firmware's kinematics tables are
# generated from the vendored URDF, so if the workspace's model changes and
# nobody regenerates, the board is measuring a DIFFERENT ROBOT from the one the
# CAD describes - every foot position it reports belongs to a leg that does not
# exist, and nothing else in the build would notice.
#
# CI checks the tables against the vendored copy and the pin beside it, which
# catches hand-edited tables and a half-done refresh. It cannot check the
# vendored copy against the workspace's live one, because that needs a
# credential for a private repo owned by another account - and GitHub's
# fine-grained tokens cannot be scoped to one of those at all.
#
# So this runs where the credential already exists: a laptop with both clones.
# Run it after pulling the workspace.
#
#   tools/model/check_drift.sh                      # finds the sibling clone
#   tools/model/check_drift.sh /path/to/workspace   # or say where it is
#
# Exit 0 = in step. Exit 1 = drifted, and it prints what to do.
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENDORED="$ROOT/tools/model/zeus.urdf"
PIN="$ROOT/tools/model/zeus_urdf.sha256"

WS="${1:-}"
if [ -z "$WS" ]; then
    for c in "$ROOT/../zeus_26" "$HOME/zeus_26" "$HOME/Downloads/zeus_26"; do
        [ -d "$c" ] && WS="$c" && break
    done
fi

if [ -z "$WS" ] || [ ! -d "$WS" ]; then
    echo "cannot find the workspace clone - pass its path as the first argument"
    exit 1
fi

UPSTREAM="$WS/zeus_description/urdf/zeus.urdf"
for f in "$VENDORED" "$PIN" "$UPSTREAM"; do
    if [ ! -f "$f" ]; then
        echo "missing: $f"
        exit 1
    fi
done

have=$(sha256sum "$VENDORED" | cut -d' ' -f1)
pin=$(tr -d '[:space:]' < "$PIN")
want=$(sha256sum "$UPSTREAM" | cut -d' ' -f1)
tables=$(grep -m1 -o 'sha256  [0-9a-f]*' "$ROOT/Appli/App/zeus_kinematics_model.h" | cut -d' ' -f3)

echo "workspace : $WS"
echo "  upstream  $want"
echo "  vendored  $have"
echo "  pin       $pin"
echo "  tables    $tables"
echo

fail=0
if [ "$have" != "$pin" ]; then
    echo "MISMATCH: the vendored URDF and the pin beside it disagree."
    fail=1
fi
if [ "$have" != "$tables" ]; then
    echo "MISMATCH: the generated tables did not come from the vendored URDF."
    fail=1
fi
if [ "$want" != "$have" ]; then
    echo "DRIFTED: the workspace's model has moved on and the firmware has not."
    fail=1
fi

if [ "$fail" -eq 0 ]; then
    echo "in step: the firmware is walking the workspace's current robot."
    exit 0
fi

cat <<REFRESH

To bring them back in step, all four steps together - a model refreshed without
regenerating, or a regeneration without re-pinning, fails one of the checks
above:

  cp "$UPSTREAM" tools/model/
  sha256sum tools/model/zeus.urdf | cut -d' ' -f1 > tools/model/zeus_urdf.sha256
  env -u PYTHONPATH ~/kinvenv/bin/python tools/gen_kinematics.py
  tools/hosttest/run.sh

Then look at what moved before committing: a new export can change contact
points, the IMU position or a joint axis, and the estimator believes all three.
REFRESH
exit 1
