---
name: bank-sim-check
description: Build and fly the bank flight mode (developed in the Betaflight repo) in this simulator. Use whenever bank mode code changes in /workspaces/betaflight (flight/bank_mode*, pg/bank_mode*, any BANK_MODE hook), when scripts/bank-*.sh, patches/bank/, src/native/bf/bf_bank.c or tests/bank-thresholds.json change, or when asked to test, check or fly bank mode in the sim.
---

# Bank mode in the simulator

The owner's how-to (build, headless check, flying it in the browser, troubleshooting) is `BANK-MODE.md` at the repo root. Point the owner there and keep it current when commands, keys or URLs change.

The sim compiles a pinned Betaflight 4.5.1 (`vendor/betaflight`). The bank mode lives on a branch of current master in `$BF_BANK_SRC` (default `/workspaces/betaflight`). We do not port the sim forward. We overlay the feature onto 4.5.1:

| Piece | Where | Maintained by |
|---|---|---|
| `flight/bank_mode.[ch]`, `pg/bank_mode.[ch]` | copied from the BF working tree into `build/bank/` by `scripts/bank-sync.sh` | nobody, automatic (uncommitted edits included) |
| master vs 4.5.1 API differences inside those files | `patches/bank/compat.sed` (sed over the staged copy), `-DRP_AXIS_COUNT=2` in `build-wasm.sh` | you, when the build breaks on a missing API |
| hooks into rc.c, pid.c, mixer.c, rc_modes.h, runtime_config.h, pg_ids.h | `patches/bank/0001-bank-mode-hooks.patch`, hand ported to 4.5.1 | you, when `bank:sync` reports DRIFT |
| core.c's part (BOXBANK switch, `bankModeUpdate()` per RC frame) | `src/native/bf/bf_bank.c` (core.c is not compiled here) | you, when the branch changes how core.c drives the mode |
| `bank_*` CLI settings | `src/native/bf/bf_settings.c` under `SIM_BANK_MODE` | you, when `bankModeConfig_t` fields change (compile error tells you) |
| accelerometer | `sim.c` measures specific force; `bf_bank.c` turns it into ICM-42688 counts for BF's own `acceleration.c` | rarely |

Everything is behind `BANK=1` / `SIM_BANK_MODE`. The default build and `dist/sim.wasm` must stay byte identical.

## One iteration

```bash
cd /workspaces/betaflight && make test_bank_unittest      # firmware unit tests first
cd /home/devpod/WebFPVSimulator && npm run bank:cycle      # sync, build sim-bank.wasm, vendor clean check, fly
```

`bank:cycle` exits non-zero on the first problem and says what it was. `npm run check:bank -- --explore` prints the time series of the sortie, which is how to see *why* a row failed. The report is in `build/bank/report.json`.

## The owner's flights

The checks fly scripted sticks; the owner flies real ones, and twice found problems no sortie provoked (lift-stick feedforward kicking the pitch, and the 1.62 g world reading as climbing). With `?fw=bank` the page records every flight and posts it to `testing-flight-logs/auto/` once a second (BANK-MODE.md, "Flight logs, automatically"). When the owner reports something:

1. `npm run bank:log` summarises the newest log: find the half seconds marked `pitch swings`, and compare the pitch rate against the firmware's pitch target and the `debug[]` columns (`DEBUG_BANK`).
2. `npm run bank:log -- --rows FROM TO` for the frames around it.
3. `npm run bank:log -- FILE --replay` flies those sticks headless through the current `dist/sim-bank.wasm`. If it reproduces, fix the firmware and replay again; if it does not, the session differs from the harness (gravity, start on the ground, settings), find out how before changing code.

Keep what was learnt from a log in PROGRESS.md with the file name; the auto folder is gitignored and does not survive a rebuild of the container.

When `bank:sync` reports DRIFT: read `git -C $BF_BANK_SRC diff master -- src/main/<file>`, port the change into the 4.5.1 patch (edit the vendor file, `git -C vendor/betaflight diff > patches/bank/0001-bank-mode-hooks.patch`, `git -C vendor/betaflight checkout -- .`), then `bash scripts/bank-sync.sh --accept`. Never accept without porting.

## Hard rules

1. **The firmware only sees what hardware gives it**: gyro, accelerometer, attitude, RC, arm state. Never feed `bank_mode.c` plant velocity, true drag, true thrust or anything else a flight controller cannot measure. A sim change that helps the mode must be one a real quad also has. The checker (`bank-check.js`) may read plant truth: it is the stopwatch, not the pilot.
2. `vendor/betaflight` is clean after every build (`git diff --stat vendor/betaflight` empty).
3. Never edit `tests/bank-thresholds.json` (or `tests/thresholds.json`) to make a row pass. The bands come from the spec in the Betaflight repo's `project_plan.md`. If a band is wrong, argue it in PROGRESS.md with the measured value and let the owner decide.
4. Fix failures in the firmware (the Betaflight repo), not by changing the sim, unless the sim is demonstrably unrealistic. Then say so and ask the owner first.
5. After touching `sim.c`, `plant.c` or anything outside `#ifdef SIM_BANK_MODE`, run `npm run build:wasm` and confirm `sha256sum dist/sim.wasm` is unchanged, then follow the `verify-flight-model` skill.
6. A new scenario goes in `scripts/bank-check.js` with a band derived from the spec, and must be shown to fail when the behaviour it checks is broken (temporarily break `bank_mode.c`, watch it go red, restore).

## Known limits (state them in every report)

- Attitude is the plant's true attitude, not `imu.c`'s estimate. On hardware sustained drag biases the estimate, so results here are optimistic about attitude.
- No accelerometer vibration or residual bias yet (white noise and quantisation only).
- The bank accelerometer reads in the local g (`bf_bank.c`): a craft at rest reads 1 g whatever the Weight setting. The firmware has no 1 g calibration, so a real sensor's scale error is not modelled or corrected.
- The checks fly the five inch only (1.62 g, from the ground). The whoop flies at 1.0 g in its room's terms and is not checked.
- No aux channels: bank is switched by `sim_set_bank_mode` (shell: `?fw=bank`, then B in flight).
- Browser checks (`lint:shell`, `lint:input`, `shots.js`) need an X display for WebGL. The devcontainer's `SIM_CHROME_BIN` (`/workspaces/betaflight/.devcontainer/chromium-xvfb.sh`) starts Xvfb on `:99` itself. If `echo $SIM_CHROME_BIN` shows `/usr/bin/chromium` instead, run `bash /workspaces/betaflight/.devcontainer/start-xvfb.sh` before the first check; see BANK-MODE.md if they time out on `__shellReady`. They had 8 existing failures on 2026-10-14 unrelated to bank (listed in BANK-MODE.md). The bank loop itself does not need a browser.

## Reporting

List each banded row as pass or fail with its measured value, any drift handled, and the known limits. Close with "harness result as above, feel awaiting the owner's flight" (`npm run serve`, open `/?fw=bank`, press B). Never claim feel is verified.
