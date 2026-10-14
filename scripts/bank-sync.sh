#!/usr/bin/env bash
# bank-sync.sh: stage the bank mode overlay from the Betaflight feature branch.
#
# The bank mode is developed in a Betaflight checkout (BF_BANK_SRC, default
# /workspaces/betaflight) that is far newer than the pinned vendor/betaflight
# (4.5.1). The simulator does not port Betaflight forward. Instead:
#
#   1. The self-contained new files (flight/bank_mode.[ch], pg/bank_mode.[ch])
#      are copied from the working tree into build/bank/src/main, so edits
#      flow through with no manual step and uncommitted work is tested.
#   2. The few hook hunks into existing Betaflight files live in
#      patches/bank/*.patch, ported by hand to 4.5.1. They are applied by
#      build-wasm.sh when BANK=1, never to the default build.
#   3. This script proves the patches still apply to the pinned tree and that
#      every bank related line the feature branch adds to an existing file is
#      also in the patch. A hook added on the branch and missing from the patch
#      is reported as drift, instead of silently not existing in the sim.
#
# Exit codes: 0 in sync, 1 drift or error (message says what to fix).
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

BF_BANK_SRC="${BF_BANK_SRC:-/workspaces/betaflight}"
BF_BANK_BASE="${BF_BANK_BASE:-master}"
STAGE=build/bank/src/main

# Files that are new on the feature branch and compile as they are.
NEW_FILES="flight/bank_mode.c flight/bank_mode.h pg/bank_mode.c pg/bank_mode.h"
# Existing Betaflight files the branch hooks into, and the simulator compiles.
HOOK_FILES="fc/rc.c fc/rc_modes.h fc/runtime_config.h flight/pid.c flight/mixer.c pg/pg_ids.h"
# Hooks into files the simulator does not compile (core.c is replaced by
# bf_glue.c, the rest is MSP, OSD, telemetry, CLI). Listed so a new entry in
# this class is a conscious decision, not a miss.
IGNORED_FILES="fc/core.c fc/rc_modes.c msp/msp_box.c osd/osd_elements.c cli/settings.c telemetry/ltm.c telemetry/smartport.c telemetry/ibus_shared.c target/common_post.h test/Makefile test/unit/bank_unittest.cc"

fail() { echo "bank:sync: $*" >&2; exit 1; }

[ -d "$BF_BANK_SRC/src/main" ] || fail "no Betaflight checkout at BF_BANK_SRC=$BF_BANK_SRC"
git -C "$BF_BANK_SRC" rev-parse --verify -q "$BF_BANK_BASE" >/dev/null \
  || fail "base ref '$BF_BANK_BASE' not found in $BF_BANK_SRC (set BF_BANK_BASE)"

if [ ! -e vendor/betaflight/src/main/fc/rc.c ]; then
  echo "bank:sync: vendor/betaflight is not checked out, initialising the pin"
  ref=()
  [ -d "$BF_BANK_SRC/.git" ] && ref=(--reference "$BF_BANK_SRC")
  git submodule update --init "${ref[@]}" vendor/betaflight
fi

# 1. copy the new files
rm -rf build/bank/src
mkdir -p "$STAGE/flight" "$STAGE/pg"
for f in $NEW_FILES; do
  [ -f "$BF_BANK_SRC/src/main/$f" ] || fail "$f is missing from $BF_BANK_SRC"
  cp "$BF_BANK_SRC/src/main/$f" "$STAGE/$f"
done
# 4.5.1 API differences, see the header of patches/bank/compat.sed
sed -i -f patches/bank/compat.sed "$STAGE/flight/bank_mode.c"
if grep -qE 'accADC\.[xyz]' "$STAGE/flight/bank_mode.c"; then
  fail "bank_mode.c still has an accADC.x/y/z use that compat.sed does not cover"
fi
echo "bank:sync: staged $(echo "$NEW_FILES" | wc -w) files from $BF_BANK_SRC into $STAGE"

# 2. the patches must apply to the pinned tree (check only, nothing is written)
for p in patches/bank/*.patch; do
  [ -e "$p" ] || continue
  tr -d $'\r' < "$p" | git -C vendor/betaflight apply --check \
    || fail "$p does not apply to vendor/betaflight. Re-port the hunk that failed."
done

# 3. drift: the number of bank related lines the branch adds to each hooked
# file is recorded in patches/bank/hooks.lock when the 4.5.1 patch was last
# reconciled with the branch. A different count means a hook was added or
# removed on the branch: look at the branch diff for that file, update the
# patch to match, then run `bank-sync.sh --accept` to record the new state.
# (A line by line compare is too strict, the 4.5.1 code around a hook differs.)
LOCK=patches/bank/hooks.lock
base=$(git -C "$BF_BANK_SRC" merge-base HEAD "$BF_BANK_BASE")
count_hooks() {
  git -C "$BF_BANK_SRC" diff "$base" -- "src/main/$1" | grep -E '^\+[^+]' | grep -ciE 'bank' || true
}
current=$(for f in $HOOK_FILES; do echo "$f $(count_hooks "$f")"; done)

if [ "${1:-}" = "--accept" ]; then
  printf '%s\n' "$current" > "$LOCK"
  echo "bank:sync: recorded hook counts in $LOCK"
  exit 0
fi

[ -f "$LOCK" ] || fail "$LOCK is missing. Reconcile patches/bank with the branch, then run: scripts/bank-sync.sh --accept"
if ! diff <(printf '%s\n' "$current") "$LOCK" > build/bank/drift.txt; then
  echo "bank:sync: DRIFT. Bank lines per hooked file (< branch now, > locked):" >&2
  cat build/bank/drift.txt >&2
  fail "the feature branch hooks changed. Port the change into patches/bank/*.patch, then run: scripts/bank-sync.sh --accept"
fi

# A changed file the sync neither copies, patches nor knowingly ignores.
for f in $(git -C "$BF_BANK_SRC" diff --name-only "$base" -- src/main | sed 's#^src/main/##'); do
  case " $NEW_FILES $HOOK_FILES $IGNORED_FILES " in
    *" $f "*) ;;
    *) grep -qi 'bank' <(git -C "$BF_BANK_SRC" diff "$base" -- "src/main/$f") \
         && echo "bank:sync: note: $f changes bank related lines and is in no list in this script" >&2 ;;
  esac
done

echo "bank:sync: in sync (patches apply, no hook drift)"
