# Bank mode in the simulator

How to build, check and fly the bank flight mode. The mode itself is developed in the Betaflight repo (`/workspaces/betaflight`, branch `banked-mode-init`, spec in its `project_plan.md`). The simulator layers it over its pinned Betaflight 4.5.1. How that layering works and the rules for changing it are in `.claude/skills/bank-sim-check/SKILL.md`.

Run everything inside the devcontainer.

## 1. Headless check (no browser, about 10 seconds)

```bash
cd ~/WebFPVSimulator
npm run bank:cycle
```

The sortie flies the five inch as the shell does, at its own gravity (1.62 g) and from the ground: it engages Bank on the ground, takes off with the lift stick, then hovers, cruises, turns, head checks and brakes.

This copies the bank mode from your Betaflight working tree (uncommitted edits included), builds `dist/sim-bank.wasm`, checks `vendor/betaflight` is still clean, and flies a scripted sortie: hover, cruise, turn, head check, brake. It prints one row per measurement:

- `PASS` / `FAIL` against the band in `tests/bank-thresholds.json`. The bands come from the spec, not from a measured run.
- `info` for numbers that have no band.

It exits non-zero on any failure. The full report is in `build/bank/report.json`.

To see what the quad did over time, rather than just the verdicts:

```bash
npm run check:bank -- --explore
```

That prints a row every 0.2 s: phase, speed, vertical speed, roll and pitch against their targets, heading against track, collective, thrust to weight (the fixed `bank_thrust_to_weight`), and the accelerometer reading in g.

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
| Throttle | Speed. The lower half is a forward speed: bottom is stopped, the centre is `bank_mid_speed` (about 20 m/s on the five inch). Only the bottom third brakes when you are faster than the stick's speed; between a third and the centre the craft coasts down to it. The upper half is a forward force, from what the centre gives to full forward force at the top: the same top speed as at full throttle in acro (about 40 m/s). Braking tilts back no further than `bank_max_brake_angle`. |
| Yaw | Look around (head check). The nose turns, the direction of travel does not, and it springs back when centred. |
| Pitch | Lift: the vertical force. Centre is 1 g, which balances the weight, so there is no vertical acceleration: a climb or a sink carries on until you stop it, and the height is not held. Bottom is no lift, top is full thrust as lift. Above the centre the stick is gentle at first and steepens towards the top. |
| Roll | Bank left or right, which turns the craft. Where the nose points in the turn is `bank_look_turn`: 0 (the default since 2026-10-17) is look dot, along the direction of travel; 100 is look turn, into the turn like a drifting car. The flight controller screen does not offer `bank_*` settings, so change the default in the Betaflight branch's `src/main/pg/bank_mode.c` and run `npm run bank:cycle`. Full stick is as steep as the craft can hold a coordinated turn: about 75 degrees at speed on the five inch, less when slow. |

When the thrust runs out, lift is kept first, then the bank, and the forward speed gives way first. A quarter of the spare thrust is always kept back so the craft can still be steered at the limit. Take off with the lift stick: at the centre the craft only just holds its weight.

Bank needs a gamepad or radio. Keys are on or off, so they cannot hold a speed or a bank angle. B is a keyboard key either way: the sim reads only four stick channels, so no aux switch is mapped yet.

After a firmware edit, run `npm run build:wasm:bank` again and reload the page.

### Flight logs, automatically

With the bank firmware loaded, every flight is recorded and the page posts the new rows to the dev server every second. They land in `testing-flight-logs/auto/webfpv-bank-<date>-<time>.csv`, one file per page load (a new one if the flight log is turned off and on in Settings). The folder is gitignored. Add `&autolog=0` to the URL to turn it off.

The file is blackbox_decode CSV (sticks, gyro, motors, pack voltage, one row per rendered frame) plus bank columns: the firmware's roll and pitch targets, yaw rate, collective, thrust to weight (the fixed setting) and accelerometer, then the plant's true attitude, altitude, vertical speed and ground speed, then `debug[0]` to `debug[7]` from the firmware's `DEBUG_BANK` mode: speed estimate and target (0.001 g; the target stops at the mid speed, the upper half of the stick is a force), the fast thrust trim (0.001, 1000 = none; the thrust the collective is modelled to make is multiplied by it, so above 1000 the props make more than modelled and below it less, as in fast forward flight), the forward force command (0.001 g), thrust to weight (0.01), turn gain (0.1), ball (0.001 g) and the vertical force command (0.001 g). The field list is at the `DEBUG_SET` calls in `flight/bank_mode.c`. Logs from 2026-10-16 on end with the plant's true direction of travel, `track (deg)`, so `yaw - track` is where the nose really pointed. `--rows` prints it with the yaw rate target as `yaw ... nose ...`. Only `npm run serve` takes the upload, so restart it after pulling this change. A server or deploy without the route answers with an error and the page stops posting.

### Reading a flight log

```bash
npm run bank:log                          # the newest log, half a second per row while Bank is on
npm run bank:log -- path/to/log.csv       # a given log
npm run bank:log -- --rows 12 14 4        # every 4th row between 12 s and 14 s, every column that matters
npm run bank:log -- --rows 12 14 4 --replay   # the same rows of the replayed flight
npm run bank:log -- --replay              # fly the log's sticks through dist/sim-bank.wasm and summarise that too
npm run bank:log -- --from 83 --replay    # start the log (and the replay, from the ground) at 83 s
```

In the summary, `gyroP span dirs` is how far the pitch rate swung and how often it changed direction: a large span is an oscillation or a kick. `pTgt span` is how far the firmware's own pitch target moved in the same half second. A pitch rate that swings while the target stays put (marked `pitch swings, target steady`) is the attitude loop or the plant reacting; one that swings with the target is the firmware commanding it, and the debug columns (speed estimate, thrust trim, thrust to weight) show which estimate moved.

`--replay` flies the logged sticks headless at the five inch's gravity from the ground, switching Bank where the log did, through whatever firmware `dist/sim-bank.wasm` was last built from. When the craft crashed in the flight, the page reset it and the replay does not, so start the replay after the reset with `--from` (the time the log's altitude came back to 0). It does not follow the logged flight exactly, small differences grow, but it reproduces what those sticks provoke, so a fix can be tried against the flight that showed the problem: edit the firmware, `npm run bank:cycle`, then `npm run bank:log -- FILE --replay` again.

The auto folder is gitignored and lives in the container, so it does not survive a rebuild. Copy a log that matters somewhere safe and note what it showed in PROGRESS.md.

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
- The accelerometer reads in the local g: the shell flies the five inch at 1.62 g, and a craft at rest reads 1 g, as a heavier quad on Earth would. The firmware has no 1 g calibration of its own, so on hardware a few percent of accelerometer scale error is not corrected.

How it feels to fly is never verified by the checks. That is your call when you fly it.
