#!/usr/bin/env bash
# sync-bank-mode.sh: copy the bank mode sources from the banked-mode-init
# fork into src/native/bf/bank/, adapted for the vendored Betaflight 4.5.1.
#
# The fork is read only here: this reads blobs with git show and never
# checks out, fetches or writes to it. The four files are compiled from
# src/native/bf/bank/ (build-wasm.sh adds it to the include path), and the
# hooks into 4.5.1's own files are patches/0003-bank-mode-hooks.patch.
#
# Usage: scripts/sync-bank-mode.sh [fork-dir] [ref]
#   fork-dir defaults to ../betaflight, ref to banked-mode-init. Pass the ref
#   WORKTREE to read the fork's working tree instead, for changes the owner
#   has not committed yet (still a plain read, nothing is written there).
#
# The two adaptations are the only differences from the fork, and each is a
# 4.5.1 API difference, not a behaviour change:
#   acc.accADC.x/.y/.z  4.5.1 has accADC as float[3]       -> accADC[X] / [Y] / [Z]
#   RP_AXIS_COUNT       not defined in 4.5.1                -> FD_YAW (== 2)
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

FORK="${1:-../betaflight}"
REF="${2:-banked-mode-init}"
OUT=src/native/bf/bank

mkdir -p "$OUT/flight" "$OUT/pg"
for f in flight/bank_mode.c flight/bank_mode.h pg/bank_mode.c pg/bank_mode.h; do
  { if [ "$REF" = WORKTREE ]; then cat "$FORK/src/main/$f"; else git -C "$FORK" show "$REF:src/main/$f"; fi; } \
    | sed -e 's/acc\.accADC\.x/acc.accADC[X]/' \
          -e 's/acc\.accADC\.y/acc.accADC[Y]/' \
          -e 's/acc\.accADC\.z/acc.accADC[Z]/' \
          -e 's/RP_AXIS_COUNT/FD_YAW/' \
    > "$OUT/$f"
done
echo "sync-bank-mode: $REF at $(git -C "$FORK" rev-parse --short HEAD) -> $OUT"
