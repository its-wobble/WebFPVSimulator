#!/usr/bin/env bash
# build-wasm.sh: compile the physics module to dist/sim.wasm with Emscripten.
#
# Betaflight IS compiled in: its controller objects are listed below and
# linked alongside src/native. The vendor tree (Betaflight 4.5.1) is COPIED
# into build/bf-src and the patches in patches/ are applied to that copy, so
# vendor/betaflight is never modified, not even for the length of a build,
# and `git diff --stat vendor/betaflight` stays empty.
#
# BANK mode comes from the owner's fork (its-wobble/betaflight, branch
# banked-mode-init), read from BANK_BF_DIR, default ../betaflight. Its four
# new files are copied over the 4.5.1 tree unchanged on every build, so an
# edit in the fork's working tree flies on the next build. Its hooks into
# rc.c, pid.c and mixer.c are patches/0003, re-fitted to 4.5.1 by hand
# because the fork is based on 2026 master and the context lines moved.
#
# Determinism flags are load-bearing: no fast math, no fp contraction, no
# relaxed SIMD. Do not remove them.
#
# This file is part of WebFPVSimulator.
#
# WebFPVSimulator is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or (at
# your option) any later version.
#
# WebFPVSimulator is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY, without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with WebFPVSimulator. If not, see <https://www.gnu.org/licenses/>.

set -euo pipefail
cd "$(dirname "$0")/.."

if ! command -v emcc >/dev/null 2>&1; then
  for d in "${EMSDK:-}" "$HOME/emsdk" /opt/emsdk; do
    if [ -n "$d" ] && [ -f "$d/emsdk_env.sh" ]; then
      # shellcheck disable=SC1091
      source "$d/emsdk_env.sh" >/dev/null 2>&1 || true
      break
    fi
  done
fi

if ! command -v emcc >/dev/null 2>&1; then
  echo "build:wasm: emcc not found. Install emsdk (https://emscripten.org) or set EMSDK." >&2
  exit 1
fi

mkdir -p dist build/obj

# The patches: 0001 adds the rotor telemetry the plant feeds the RPM
# filter, 0002 resets runtime statics on init so a re-init is a real reset
# rather than a warm start, and 0003 is BANK mode's hooks.
BANK_BF_DIR="${BANK_BF_DIR:-../betaflight}"
BANK_FILES="flight/bank_mode.c flight/bank_mode.h pg/bank_mode.c pg/bank_mode.h"
for f in $BANK_FILES; do
  if [ ! -f "$BANK_BF_DIR/src/main/$f" ]; then
    echo "build:wasm: $BANK_BF_DIR/src/main/$f not found. Set BANK_BF_DIR to a checkout" >&2
    echo "build:wasm: of its-wobble/betaflight banked-mode-init." >&2
    exit 1
  fi
done
BANK_REV=$(git -C "$BANK_BF_DIR" describe --always --dirty 2>/dev/null || echo unknown)
echo "build:wasm: BANK mode from $BANK_BF_DIR at $BANK_REV"

# Wiped every build so a patch that stops applying fails here rather than
# compiling a half patched tree.
BF=build/bf-src
rm -rf "$BF"
mkdir -p "$BF/src"
cp -R vendor/betaflight/src/main "$BF/src/main"
for f in $BANK_FILES; do
  cp "$BANK_BF_DIR/src/main/$f" "$BF/src/main/$f"
done
# Strip CR before apply. Windows core.autocrlf checks the patch files out
# as CRLF, and git apply then fails to match the LF tree. Run from the
# repository root with --directory: build/ is inside this repository, and
# git apply run from inside it resolves paths from the root and silently
# skips every file.
for p in patches/*.patch; do
  [ -f "$p" ] || continue
  tr -d $'\r' < "$p" | git apply --directory="$BF" -
done

CFLAGS_COMMON="-std=gnu17 -O2 -fno-fast-math -ffp-contract=off"

# Simulator sources, plain includes.
SIM_SRC="src/native/sim.c src/native/plant.c src/native/bridge.c src/native/world.c src/native/libm/sim_math.c"

# Betaflight sources compiled with the SITL target configuration, plus the
# glue that feeds the simulated gyro in and reads motor outputs back.
# SIM_ROTOR_TELEMETRY is read by patches/0001: it switches on the dynamic
# notch, the RPM filter and dynamic idle. All three need DShot telemetry on
# hardware to learn rotor speed, and this simulator knows it exactly, so the
# only thing standing between them and working was a target guard.
# USE_BANK_MODE is defined here rather than through the fork's
# common_post.h hunk, which gates it on the flash size of a real target.
BF_INC="-I $BF/src/main/target/SITL -I $BF/src/main -DSIMULATOR_BUILD -DSIM_ROTOR_TELEMETRY -DUSE_BANK_MODE"
BF_SRC="
  src/native/bf/bf_glue.c
  src/native/bf/bf_settings.c
  src/native/bf/bf_stubs.c
  $BF/src/main/sensors/gyro.c
  $BF/src/main/sensors/gyro_init.c
  $BF/src/main/sensors/boardalignment.c
  $BF/src/main/config/simplified_tuning.c
  $BF/src/main/fc/rc.c
  $BF/src/main/fc/rc_controls.c
  $BF/src/main/fc/rc_modes.c
  $BF/src/main/fc/controlrate_profile.c
  $BF/src/main/fc/runtime_config.c
  $BF/src/main/flight/pid.c
  $BF/src/main/flight/pid_init.c
  $BF/src/main/flight/mixer.c
  $BF/src/main/flight/mixer_init.c
  $BF/src/main/flight/dyn_notch_filter.c
  $BF/src/main/flight/rpm_filter.c
  $BF/src/main/flight/bank_mode.c
  $BF/src/main/pg/bank_mode.c
  $BF/src/main/common/maths.c
  $BF/src/main/common/filter.c
  $BF/src/main/common/sdft.c
  $BF/src/main/common/bitarray.c
  $BF/src/main/pg/rx.c
  $BF/src/main/pg/motor.c
  $BF/src/main/pg/dyn_notch.c
  $BF/src/main/pg/rpm_filter.c
  $BF/src/main/config/feature.c
  $BF/src/main/build/debug.c
"

OBJS=""
for src in $SIM_SRC; do
  obj="build/obj/$(basename "$src" .c).o"
  emcc -c "$src" -I src/native $CFLAGS_COMMON -Wall -o "$obj"
  OBJS="$OBJS $obj"
done
# Object names carry the parent directory. Betaflight has more than one file
# with the same basename (flight/rpm_filter.c and pg/rpm_filter.c), and a
# basename-only rule silently overwrites one with the other, which the linker
# then reports as a pile of duplicate symbols from a single object.
for src in $BF_SRC; do
  obj="build/obj/bf_$(basename "$(dirname "$src")")_$(basename "$src" .c).o"
  emcc -c "$src" -I src/native $BF_INC $CFLAGS_COMMON -o "$obj"
  OBJS="$OBJS $obj"
done

emcc $OBJS \
  --no-entry \
  -sSTANDALONE_WASM=1 \
  -sEXPORTED_FUNCTIONS=_malloc,_free \
  -sALLOW_MEMORY_GROWTH=1 \
  -o dist/sim.wasm

echo "build:wasm: wrote dist/sim.wasm"
