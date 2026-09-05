#!/usr/bin/env bash
#
# Host tests for the parts of Appli/App that do not need a board.
#
# Much of the App layer is pure logic - bit maps, debounce, command
# validation, the Lie-group maths - and all of it is currently only ever
# exercised by flashing a robot. These build and run on a workstation in under
# a second, so a mistake in that logic is caught before it reaches hardware.
#
#     tools/hosttest/run.sh
#
# Exits non-zero if any test fails, so it can gate a commit or a CI job.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
APP="$ROOT/Appli/App"
OUT="$HERE/build"
CC="${CC:-cc}"

# -Wconversion is deliberate: C1 (a joint index silently narrowed into a
# bitmask field) is exactly the class of bug it catches.
# -Wconversion and -Wshadow are deliberate, and -Werror keeps them honest.
# C1 - a joint index silently narrowed into a bitmask field - is exactly the
# class of bug the first catches, and the second found an estimator loop index
# shadowing the accelerometer vector in the same function.
CFLAGS="-std=c11 -O1 -g -Wall -Wextra -Wconversion -Wshadow -Werror
        -I$HERE/stub -I$APP"
LDLIBS="-lm"

mkdir -p "$OUT"
rm -f "$OUT"/*

fail=0

run_suite() {
    local name="$1"; shift
    local bin="$OUT/$name"

    # Skip a suite whose sources are not present, so the runner still works
    # on a checkout that predates it.
    for src in "$@"; do
        if [ ! -f "$src" ]; then
            echo "=== skipping $name (missing $(basename "$src")) ==="
            echo
            return
        fi
    done

    echo "=== building $name ==="
    if ! $CC $CFLAGS -o "$bin" "$@" $LDLIBS 2>&1; then
        echo "  BUILD FAILED"
        fail=1
        return
    fi

    echo "=== running $name ==="
    if ! "$bin"; then
        fail=1
    fi
    echo
}

run_suite test_lie_group \
    "$HERE/test_lie_group.c" \
    "$APP/lie_group.c"

run_suite test_contact \
    "$HERE/test_contact.c" \
    "$HERE/stub/hal_stub.c" \
    "$APP/contact.c"

run_suite test_safety \
    "$HERE/test_safety.c" \
    "$APP/safety.c"

run_suite test_fusion \
    "$HERE/test_fusion.c" \
    "$APP/fusion.c" \
    "$APP/inekf.c" \
    "$APP/lie_group.c" \
    "$APP/kinematics.c" \
    "$APP/robot_config.c"

run_suite test_robot_config \
    "$HERE/test_robot_config.c" \
    "$APP/robot_config.c"

run_suite test_watchdog \
    "$HERE/test_watchdog.c" \
    "$HERE/stub/hal_stub.c" \
    "$APP/watchdog.c"

echo "=== protocol layout (C vs Python) ==="
if ! python3 "$ROOT/tools/check_proto.py" >/dev/null; then
    echo "  check_proto.py FAILED - run it directly for the field table"
    fail=1
else
    echo "  C and Python agree"
fi
echo

if [ "$fail" -ne 0 ]; then
    echo "HOST TESTS FAILED"
    exit 1
fi

echo "HOST TESTS PASSED"
