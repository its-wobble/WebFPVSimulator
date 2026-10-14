#!/usr/bin/env bash
# bank-cycle.sh: one iteration of the bank mode loop. Sync the overlay from
# the Betaflight checkout, build dist/sim-bank.wasm, prove the vendor tree is
# clean, fly the headless scenarios. Non-zero exit on the first problem, with
# the reason on stderr. See .claude/skills/bank-sim-check/SKILL.md.
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

BANK=1 bash scripts/build-wasm.sh
if [ -n "$(git diff --stat vendor/betaflight)" ]; then
  echo "bank:cycle: vendor/betaflight is dirty after the build. A patch was not reverted." >&2
  exit 1
fi
node scripts/bank-check.js "$@"
