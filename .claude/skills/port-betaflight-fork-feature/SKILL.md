---
name: port-betaflight-fork-feature
description: Bring a feature from the owner's Betaflight fork (../betaflight, its-wobble/betaflight) into the simulator for testing without moving the vendored Betaflight 4.5.1. Use when the owner wants to test a change they made in the fork, wants to update the bank mode port after new fork commits, or wants another fork feature flown in the sim. Bank mode (branch banked-mode-init) is the worked example.
---

# Porting a fork feature into the simulator

The vendored Betaflight stays at 4.5.1 (`vendor/betaflight`, read only). The fork is a newer master snapshot plus the owner's commits, so its diff cannot be applied here. The method: copy the feature's own new files into the sim repo, port its hooks into 4.5.1 files as a patch, replace what `fc/core.c` would do (it is not compiled) with glue, and fly it. Bank mode shows each step: `src/native/bf/bank/`, `patches/0003-bank-mode-hooks.patch`, `scripts/sync-bank-mode.sh`, and `bridge_set_bank_mode` in `src/native/bf/bf_glue.c`.

## Ground rules, from CLAUDE.md

- The fork is read only. Read it with `git -C ../betaflight show <ref>:<path>` and `git diff`. Never check out, fetch into, or edit it.
- Vendor stays clean. Every change to a 4.5.1 file is a patch in `patches/`, applied by `scripts/build-wasm.sh` and reverted on exit. `git diff --stat vendor/betaflight` must be empty after a build.
- A change to the plant, the ABI or the build needs the owner's approval first: say what changes, why, and what it could break, wait, then record the answer in PROGRESS.md with the date. Ask about a sensor or plant addition before writing it.
- No em dashes or en dashes in prose, comments, commit messages or docs. GPLv3 header on every new file.
- Do not use JS `Math` trig in the physics path. Fork C code calling `sinf`, `cosf` and friends is fine: Emscripten links its own libm into the wasm. Confirm the module still imports nothing but memory growth.
- Do not run `npm run verify` casually, but a plant, ABI or build change is exactly when to run it. Always finish by asking the owner which verification scale they want (none, cheap, shots, verify, fly it).

## Steps

1. Find what the feature is. In the fork: `git branch -a`, `git merge-base banked-mode-init origin/master`, `git diff --stat <base>...<branch>`. `upstream/master` may not be fetched locally, so the base can be `origin/master`. Split the diff into three kinds of file:
   - **New files** (the feature's own logic and parameter group). These get copied.
   - **Hooks into existing files** (`pid.c`, `mixer.c`, `rc.c`, `runtime_config.h`, `pg_ids.h`). These get ported by hand into a patch.
   - **Everything else** (CLI, MSP, OSD, telemetry, `core.c`, `rc_modes`, unit tests, makefiles). Not taken, except that `core.c`'s mode gating gets re-created in `bf_glue.c`.
2. Check what 4.5.1 is missing. Compare the APIs the new files use (`acc_t` layout, `RP_AXIS_COUNT`, `getRcDeflection`, `attitude`, anything new in `pidLevel`). Each difference becomes a `sed` line in the sync script, not an edit to the copied file. Keep the copy faithful to the fork.
3. Write `scripts/sync-<feature>.sh` following `scripts/sync-bank-mode.sh`: `git show <ref>:src/main/<file> | sed ... > src/native/bf/<feature>/<file>`. Then the owner can re-run it when they push more fork commits.
4. Generate the hooks patch against the state the build sees, which is 4.5.1 with patches 0001 to 0002 already applied (patches apply in `ls` order):
   - Copy the touched vendor files into a scratch dir `a/src/main/...`, apply the existing patches there with `git apply` (strip CR: `tr -d '\r'`), copy to `b/`, edit `b/`.
   - `git -c core.autocrlf=false diff --no-index a b`, then `sed -E '/^(diff --git|---|\+\+\+) /{s#a/a/#a/#g; s#b/b/#b/#g}'` to fix the prefixes. Test with `git apply --check` on a copy of `a/`. Name it `patches/000N-<feature>-hooks.patch`.
   - Never edit `vendor/betaflight` in place to make the diff. Use the scratch copy.
5. Wire the build. In `scripts/build-wasm.sh`: the feature's `-DUSE_<FEATURE>`, `-I src/native/bf/<feature>`, and the new `.c` files in `BF_SRC`. Object names carry the parent directory, so `flight/x.c` and `pg/x.c` do not collide. Update the patch count comment.
6. Glue in `src/native/bf/bf_glue.c`, following the angle mode and launch control pattern: a `g_<feature>` flag that defaults to 0 so the acro path never touches the feature, a `bridge_set_<feature>` entry point, the flag re-applied in `bridge_reset`, and the per step feed in `bridge_run`. Add storage the uncompiled files would have defined to `bf_stubs.c`, the parameter group reset in `bf_runtime_init`, and the CLI keys in `bf_settings.c`. Expose `sim_set_<feature>` in `sim.c` and `sim_abi.h` (additive, no ABI version bump) and `setX` in `tests/lib/simmod.js` as a method, not in the required export list.
7. If the feature needs a signal the plant does not produce (bank mode needed an accelerometer), compute it inside `plant_step` from forces already summed there, store it in a new `SimState` field, and keep it out of the exported state array so no replay hash moves. Ask the owner first.
8. Shell wiring. Bank mode is a third value of the **Flight mode** row on the **Quad** screen (not Settings), with `main.js` `syncAngleMode` driving the ABI call. New CLI keys also need `scripts/fc-catalog-gen.js` (a `FORK_ONLY` entry with type and limits from the fork's `cli/settings.c`), a `PG_TAB` entry in `src/fc/catalog.js`, and `node scripts/fc-catalog-gen.js` to regenerate `catalog-data.js`.
9. Build and prove nothing else moved:
   - Install emsdk first: `source ~/emsdk/emsdk_env.sh`. This machine has 6.0.12; PROGRESS.md recorded 3.1.61 earlier, so a fresh build is not byte identical to an older committed `dist/sim.wasm`.
   - `npm run build:wasm`, then `git diff --stat vendor/betaflight` (empty).
   - A/B: `git stash -u`, build, copy `dist/sim.wasm` aside, `git checkout dist/sim.wasm`, `git stash pop`, build again. Run the same scripted input stream through both modules in Node (`tests/lib/simmod.js`, `readStateBytes`, sha256) in acro and in angle mode. The hashes must match. Only the new mode may differ.
   - Fly the new mode headlessly in Node and print altitude, speed, attitude and heading. Mind that in the ABI pitch channel + is nose up, so a forward push is negative.
10. Cheap checks: `npm run lint:fc`, `lint:presets`, `lint:catalog`, `lint:preload`. Then the `verify-flight-model` skill and `npm run verify` for the plant, ABI and build.
11. Append to PROGRESS.md: the owner's approval and what it covered, what changed, every check actually run with its measured value, what went wrong, what was not run. Say plainly that flight feel is unverified.
12. Hand it to the owner with how to fly it: `npm run serve`, open http://127.0.0.1:8000/, Quad screen, Flight section, Flight mode. Say what would count as wrong.

## Things that bit last time

- The shell is fish. Unquoted globs with no match abort the command, and `echo == x` fails. Quote them or use `bash -c`.
- A feature's tuning can be craft specific. Bank mode's `bank_thrust_to_weight` defaults to 2.0 and the whoop needs about 3.5 to 4.0 to hold a hover at centre throttle. Check the defaults against the plant before blaming the port.
- A fork hunk that "looks the same" in `pid.c` usually is not: 4.5.1's `pidLevel` has no `angleLimit` override and `acc_t` has `accADC[3]`. Read the 4.5.1 source, do not paste.
- `tests/` files, thresholds and `baseline.rec` are not edited to make a check pass. Adding a wrapper method to `tests/lib/simmod.js` is fine; changing a band is not.
- A bank change in `dist/sim.wasm` rebuilt on a different emsdk is the owner's call before it goes to main. Say so.
- Commit and push only when the owner agrees; `main` is append only.
