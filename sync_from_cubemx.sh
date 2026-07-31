#!/usr/bin/env bash
#
# Pull CubeMX-generated files from the CubeMX output folder into this repo.
#
# Why this exists: CubeMX generates into its own project folder (derived from
# Project Location + Project Name in the Project Manager tab), which is not
# this git repo. Rather than fight that, generate wherever CubeMX wants and
# then run this script to bring back only the files CubeMX actually owns.
#
#   ./sync_from_cubemx.sh                      # default source folder
#   ./sync_from_cubemx.sh /path/to/nexus_first # explicit source folder
#
# Afterwards ALWAYS review before building:
#
#   git diff --stat
#
# If Appli/App/ shows up in that diff, something went wrong - run
# "git checkout -- Appli/App" to undo it.
#
set -euo pipefail

SRC="${1:-c:/Users/visma/Downloads/nexus_first}"
DST="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Files and directories CubeMX generates and is allowed to overwrite.
# Deliberately absent:
#   Appli/App          - hand-written application code, CubeMX knows nothing of it
#   */CMakeLists.txt   - "generated only once", holds the App/*.c source list
#   */CMakePresets.json- same
#   build/             - artifacts, and stale caches cause confusing failures
#   .git/              - obviously
MANAGED=(
    "nexus_first.ioc"
    "mx-generated.cmake"
    "Appli/Core"
    "Appli/USB_DEVICE"
    "Appli/mx-generated.cmake"
    "Boot/Core"
    "Boot/mx-generated.cmake"
    "Drivers"
    "Middlewares"
)

# Linker scripts live beside the sources and are regenerated too.
MANAGED_GLOBS=(
    "Appli/*.ld"
    "Boot/*.ld"
)

if [ ! -d "$SRC" ]; then
    echo "ERROR: source folder not found: $SRC" >&2
    echo "Pass it explicitly:  ./sync_from_cubemx.sh /path/to/cubemx/project" >&2
    exit 1
fi

if [ ! -d "$DST/.git" ]; then
    echo "ERROR: $DST is not a git repo. Refusing to copy without a safety net." >&2
    exit 1
fi

if [ ! -f "$SRC/nexus_first.ioc" ]; then
    echo "ERROR: $SRC does not look like the CubeMX project (no nexus_first.ioc)." >&2
    exit 1
fi

echo "Source: $SRC"
echo "Repo:   $DST"
echo

# Fingerprint the hand-written app code so we can tell whether THIS run touched
# it. Comparing against git HEAD would be wrong - App legitimately has
# uncommitted work in it most of the time.
app_fingerprint() {
    find "$DST/Appli/App" -type f -exec md5sum {} + 2>/dev/null | sort | md5sum
}
APP_BEFORE="$(app_fingerprint)"

for path in "${MANAGED[@]}"; do
    if [ -e "$SRC/$path" ]; then
        mkdir -p "$(dirname "$DST/$path")"
        rm -rf "$DST/$path"
        cp -r "$SRC/$path" "$DST/$path"
        echo "  synced  $path"
    else
        echo "  SKIP    $path (not present in source)"
    fi
done

for glob in "${MANAGED_GLOBS[@]}"; do
    for f in $SRC/$glob; do
        [ -e "$f" ] || continue
        cp "$f" "$DST/$(dirname "$glob")/"
        echo "  synced  $(dirname "$glob")/$(basename "$f")"
    done
done

echo

if [ "$(app_fingerprint)" = "$APP_BEFORE" ]; then
    echo "OK: Appli/App untouched by this sync."
else
    echo "WARNING: this sync modified Appli/App. That should never happen."
    echo "         Undo with:  git checkout -- Appli/App"
fi

# ---------------------------------------------------------------------------
# Project-specific sanity check.
#
# Every DMA buffer sits in the "noncacheable_buffer" section at 0x24070000, but
# what actually makes that memory non-cacheable is MPU Region 2, generated into
# Boot/Core/Src/main.c. If the CubeMX folder was saved but never regenerated,
# syncing silently drags the old 0x0 value back in - and the firmware then runs
# with cached DMA buffers, which corrupts sensor data intermittently.
#
# That is not a hypothetical: it is exactly what happened the first time this
# script was run.
# ---------------------------------------------------------------------------
MPU_LINE="$(grep -A1 'MPU_REGION_NUMBER2' "$DST/Boot/Core/Src/main.c" | grep BaseAddress || true)"
if ! echo "$MPU_LINE" | grep -q '0x24070000'; then
    echo
    echo "=============================================================="
    echo "WARNING: MPU Region 2 is NOT covering the non-cacheable buffers."
    echo "  found:    ${MPU_LINE:-<no BaseAddress line>}"
    echo "  expected: MPU_InitStruct.BaseAddress = 0x24070000;  (size 8KB)"
    echo
    echo "The source folder's generated code is stale. In CubeMX open the"
    echo "Boot context -> System Core -> CORTEX_M7 -> MPU Region 2, confirm"
    echo "base 0x24070000 / size 8KB, then use Generate Code (Alt+K), NOT"
    echo "just Save. Then re-run this script."
    echo "=============================================================="
fi

echo
echo "Now review what changed before you build:"
echo "    git diff --stat -- . ':!Drivers' ':!Middlewares'"
echo
echo "Then rebuild from scratch (CubeMX may have rewritten mx-generated.cmake):"
echo "    rm -rf build Appli/build Boot/build"
echo "    cmake --preset Debug && cmake --build build/Debug"
