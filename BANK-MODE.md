# Bank mode in the simulator

How to build, check and fly the bank flight mode. The mode itself is developed in the Betaflight repo (`/workspaces/betaflight`, branch `banked-mode-init`, spec in its `project_plan.md`). The simulator layers it over its pinned Betaflight 4.5.1. How that layering works and the rules for changing it are in `.claude/skills/bank-sim-check/SKILL.md`.

Run everything inside the devcontainer.

## 1. Headless check (no browser, about 10 seconds)

```bash
cd ~/WebFPVSimulator
npm run bank:cycle
```

This copies the bank mode from your Betaflight working tree (uncommitted edits included), builds `dist/sim-bank.wasm`, checks `vendor/betaflight` is still clean, and flies a scripted sortie: hover, cruise, turn, head check, brake. It prints one row per measurement:

- `PASS` / `FAIL` against the band in `tests/bank-thresholds.json`. The bands come from the spec, not from a measured run.
- `info` for numbers that have no band.

It exits non-zero on any failure. The full report is in `build/bank/report.json`.

To see what the quad did over time, rather than just the verdicts:

```bash
npm run check:bank -- --explore
```

That prints a row every 0.2 s: phase, speed, vertical speed, roll and pitch against their targets, heading against track, collective, learnt thrust to weight, and the accelerometer reading in g.

Run the firmware unit tests first: `make test_bank_unittest` in `/workspaces/betaflight`.

## 2. Fly it in the browser

```bash
cd ~/WebFPVSimulator
npm run build:wasm:bank     # not needed if bank:cycle just ran
npm run serve
```

1. Open **http://localhost:8000/** in your own browser (VS Code forwards port 8000). Once `dist/sim-bank.wasm` exists, `npm run serve` redirects a plain visit to `?fw=bank`, so the bank firmware loads by default. For the stock firmware open `/?fw=normal`, or start the server with `SIM_FW=normal npm run serve`.
2. Start a flight.
3. Press **B** to turn Bank on. A banner explains the sticks and the on-screen mode indicator reads **Bank**. Press **B** again to go back to Acro or Angle.

Sticks in Bank (mode 2):

| Stick | Does |
|---|---|
| Throttle | Forward speed. Bottom is stopped (and brakes), top is the fastest speed. |
| Yaw | Bank left or right, which turns the craft. |
| Pitch | Lift. Centre holds 1 g (no vertical acceleration), bottom is no lift, top is full thrust as lift. |
| Roll | Look around (head check). The nose turns, the direction of travel does not, and it springs back when centred. |

Bank needs a gamepad or radio. Keys are on or off, so they cannot hold a speed or a bank angle. B is a keyboard key either way: the sim reads only four stick channels, so no aux switch is mapped yet.

After a firmware edit, run `npm run build:wasm:bank` again and reload the page.

## If something is wrong

- **B does nothing.** In the browser devtools console, `__bankSwitch(true)` returns `true` when Bank engaged and `false` when the page loaded the normal firmware. Check the URL ends in `?fw=bank` (the server adds it only when `dist/sim-bank.wasm` exists and `SIM_FW` is not `normal`). The file is gitignored, so a fresh clone has to build it. `__flightMode()` reports the live mode.
- **Bank switches itself off.** Turtle, crash flip and a launch control hold override it, the same way they override Angle.
- **`bank:sync` reports DRIFT.** A bank hook changed on the Betaflight branch. Port it into `patches/bank/` as `.claude/skills/bank-sim-check/SKILL.md` describes, then run `bash scripts/bank-sync.sh --accept`.
- **The bank build fails to compile.** `bank_mode.c` uses an API that 4.5.1 lacks. Add a mapping to `patches/bank/compat.sed` or a define in `scripts/build-wasm.sh`.
- **The browser checks time out waiting for `__shellReady`** (`lint:shell`, `lint:input`, `shots.js`). Debian's Chromium needs an X display to get WebGL, even headless. In the devcontainer `SIM_CHROME_BIN` points at `/workspaces/betaflight/.devcontainer/chromium-xvfb.sh`, which starts Xvfb on `:99` before launching Chromium, so this should not happen. If it does (an older container still has `SIM_CHROME_BIN=/usr/bin/chromium`), run `bash /workspaces/betaflight/.devcontainer/start-xvfb.sh` once, or prefix the check with `SIM_CHROME_BIN=/workspaces/betaflight/.devcontainer/chromium-xvfb.sh`. Check the display with `pgrep -a Xvfb`; its log is `/tmp/xvfb.log`.
- **`lint:shell` and `lint:input` report failures.** As of 2026-10-14 they fail the same way with or without the bank changes (5 layout overflows in `lint:shell`; the 6S pack, X key climb and track builder question rows in `lint:input`). Those are existing, not bank related. Compare against that list before blaming a bank change.

## What a result does and does not tell you

The firmware sees only what a flight controller has: gyro, an ICM-42688-style accelerometer (2048 counts per g, 1 kHz, Betaflight's own 25 Hz filter), attitude, the RC sticks and arm state. Two simplifications make results optimistic:

- Attitude is the sim's true pose, not Betaflight's estimate. On a real quad, sustained drag pulls that estimate off.
- The accelerometer has no frame vibration and no residual bias, only noise and rounding.

How it feels to fly is never verified by the checks. That is your call when you fly it.
