/*
 * bank-check.js: fly the bank flight mode headless and measure it.
 *
 * Loads dist/sim-bank.wasm (npm run build:wasm:bank), flies scripted stick
 * scenarios in bank mode at the five inch's gravity, taking off from the
 * ground as the shell does, and prints one row per measurement with
 * its band from tests/bank-thresholds.json. Exit 1 if any row fails, so an
 * agent can loop on it. Writes build/bank/report.json with every value.
 *
 * WHO SEES WHAT. The firmware only ever sees what hardware gives it (see
 * src/native/bf/bf_bank.c). This script is the observer and reads the plant's
 * truth (velocity, attitude) to judge the result, which is fine: it is the
 * stopwatch, not the pilot.
 *
 * Seconds of wall clock, no browser. `--explore` prints the time series of
 * each scenario instead of judging it, for designing a new scenario.
 *
 * This file is part of WebFPVSimulator.
 *
 * WebFPVSimulator is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at
 * your option) any later version.
 *
 * WebFPVSimulator is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WebFPVSimulator. If not, see <https://www.gnu.org/licenses/>.
 */

import { mkdir, readFile, writeFile } from 'node:fs/promises';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

import { airframeById } from '../configs/airframes.js';
import { RATE_DEFAULTS } from '../configs/rates.js';
import { composeConfig, RATES_KEEP } from '../src/fc/dump.js';
import { loadSim, SIM_OK, simErrorName } from '../tests/lib/simmod.js';
import { must, sha256Hex } from '../tests/lib/replay.js';

const root = dirname(dirname(fileURLToPath(import.meta.url)));
const EXPLORE = process.argv.includes('--explore');
const RX_HZ = 250;
const STEP_HZ = 1000;
const DEG = 180 / Math.PI;

const wasm = await readFile(join(root, 'dist/sim-bank.wasm')).catch(() => {
  console.error('bank:check: dist/sim-bank.wasm missing. Run npm run build:wasm:bank');
  process.exit(1);
});
const defaultDiff = await readFile(join(root, 'configs/betaflight-default.diff'), 'utf8');
const thresholds = JSON.parse(await readFile(join(root, 'tests/bank-thresholds.json'), 'utf8'));

/* Bank stick intent to the ABI's channels. speed 0..1 (throttle), bank -1..1
 * (yaw stick, + right), lift 0..1 (pitch stick, 0.5 centre = 1 g, 1 = stick
 * fully forward), head -1..1 (roll stick, + right). The ABI's pitch channel
 * is + nose up, which is stick BACK, hence the sign. */
function channels({ speed = 0, bank = 0, lift = 0.5, head = 0 }) {
  return { roll: head, pitch: -(2 * lift - 1), yaw: bank, throttle: speed };
}

/* Plant truth helpers. State: t, pos3, vel3, quat(wxyz)4, omega3, ... */
function eulerOf(q) {
  const [w, x, y, z] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return { roll, pitch, yaw };
}
function wrap(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

async function newFlight() {
  const sim = await loadSim(wasm);
  if (typeof sim.e.sim_set_bank_mode !== 'function') {
    throw new Error('sim-bank.wasm has no sim_set_bank_mode; was it built with BANK=1?');
  }
  const cfg = composeConfig(defaultDiff, RATE_DEFAULTS, RATES_KEEP);
  const code = sim.init(cfg);
  if (code !== SIM_OK) {
    throw new Error(`sim_init: ${simErrorName(code)}`);
  }
  must(sim.setCellVoltage(4.0), 'sim_set_cell_voltage');
  /* Flown as the shell flies it: at the five inch's own gravity (1.62 g, the
   * shell's Weight 100) and from the ground. The bank accelerometer reads in
   * the local g (bf_bank.c), so a craft at rest reads 1 g as on Earth; the
   * module's default 1.0 g in free air hid that the shell's world once read
   * 1.62 g at rest, and the owner's flights oscillated for it (PROGRESS.md,
   * 2026-10-15). */
  must(sim.e.sim_set_gravity(airframeById('5inch').gravityBase), 'sim_set_gravity');
  must(sim.reset(), 'sim_reset');
  must(sim.e.sim_set_ground(1, 0, 0, 1, 0, 0, 0, 0.55, 0.28), 'sim_set_ground');
  const telPtr = sim.e.malloc(32 * 8);
  let t = 0;
  let sticks = channels({});
  const samples = [];
  const hash = [];
  const flight = {
    sim,
    bank(on) { must(sim.e.sim_set_bank_mode(on ? 1 : 0), 'sim_set_bank_mode'); },
    sticks(s) { sticks = channels(s); },
    /* Fly `seconds` with the current sticks, sampling every RC frame. */
    fly(seconds, label = '') {
      const frames = Math.round(seconds * RX_HZ);
      for (let f = 0; f < frames; f += 1) {
        must(sim.input(t, sticks.roll, sticks.pitch, sticks.yaw, sticks.throttle), 'sim_input');
        must(sim.step(STEP_HZ / RX_HZ), 'sim_step');
        t += 1 / RX_HZ;
        const { state } = sim.readState();
        const n = sim.e.sim_bank_telemetry(telPtr, 32);
        const tel = Array.from(new Float64Array(sim.e.memory.buffer, telPtr, n));
        const e = eulerOf(state.slice(7, 11));
        const v = state.slice(4, 7);
        samples.push({
          label, t: state[0], pos: state.slice(1, 4), vel: v,
          roll: e.roll * DEG, pitch: e.pitch * DEG, yaw: e.yaw * DEG,
          speedH: Math.hypot(v[0], v[1]), track: Math.atan2(v[1], v[0]) * DEG,
          active: tel[0], rollTgt: tel[1], pitchTgt: tel[2], yawRateCmd: tel[3],
          collective: tel[4], tw: tel[5], acc: tel.slice(6, 9),
        });
        hash.push(...state.slice(0, 14));
      }
    },
    window(label, fromEnd = Infinity) {
      const s = samples.filter((x) => x.label === label);
      const n = Math.min(s.length, Math.round(fromEnd * RX_HZ));
      return s.slice(s.length - n);
    },
    samples,
    hash() { return sha256Hex([new Uint8Array(new Float64Array(hash).buffer)]); },
  };
  return flight;
}

const mean = (a) => a.reduce((s, x) => s + x, 0) / Math.max(1, a.length);
const span = (a) => Math.max(...a) - Math.min(...a);

/* The standard sortie: engage, settle, cruise, turn, head check, brake. Every
 * scenario reads its own labelled window, so one flight serves them all. */
async function sortie() {
  const f = await newFlight();
  f.sticks({ speed: 0, lift: 0.5 });
  f.bank(true);
  f.fly(1.0, 'engage');
  /* At the centre the lift stick asks for 1 g, which the default thrust to
   * weight under-delivers, so the craft sits on the ground until lifted. */
  f.sticks({ speed: 0, lift: 0.65 });
  f.fly(0.8, 'takeoff');
  /* Long enough for the vertical damping to stop the take-off climb before
   * the last 3 s, which are what the hover rows measure. */
  f.sticks({ speed: 0, lift: 0.5 });
  f.fly(6.0, 'hover');
  f.sticks({ speed: 0.5, lift: 0.5 });
  f.fly(8.0, 'cruise');
  f.sticks({ speed: 0.5, lift: 0.5, bank: 0.6 });
  f.fly(6.0, 'turn');
  f.sticks({ speed: 0.5, lift: 0.5 });
  f.fly(3.0, 'unturn');
  f.sticks({ speed: 0.5, lift: 0.5, head: 1.0 });
  f.fly(2.0, 'headcheck');
  f.sticks({ speed: 0.5, lift: 0.5 });
  f.fly(2.0, 'headreturn');
  f.sticks({ speed: 0, lift: 0.5 });
  f.fly(6.0, 'brake');
  f.sticks({ speed: 0, lift: 0.5 });
  f.bank(false);
  f.fly(1.0, 'off');
  return f;
}

/* Least squares slope of vertical speed over the window: the mean vertical
 * acceleration, robust to the 250 Hz sampling of a noisy signal. m/s^2. */
function vertAccel(w) {
  const t0 = mean(w.map((x) => x.t));
  const v0 = mean(w.map((x) => x.vel[2]));
  let num = 0;
  let den = 0;
  for (const x of w) {
    num += (x.t - t0) * (x.vel[2] - v0);
    den += (x.t - t0) ** 2;
  }
  return num / den;
}
const noseOffset = (x) => wrap((x.yaw - x.track) / DEG) * DEG;
const rate = (w, key) => (wrap((w.at(-1)[key] - w[0][key]) / DEG) * DEG) / (w.at(-1).t - w[0].t);

/* Every measurement is named for what the spec in the Betaflight repo's
 * project_plan.md promises. The bands in tests/bank-thresholds.json come from
 * that spec, not from what this run happened to measure. */
function measure(f) {
  const m = {};
  const hover = f.window('hover', 3);
  m.hover_vertical_accel = vertAccel(hover);
  m.hover_tilt_deg = Math.max(...hover.map((x) => Math.hypot(x.roll, x.pitch)));
  m.hover_horizontal_speed = mean(hover.map((x) => x.speedH));
  const cruise = f.window('cruise', 3);
  m.cruise_speed = mean(cruise.map((x) => x.speedH));
  m.cruise_speed_variation = span(cruise.map((x) => x.speedH));
  m.cruise_vertical_accel = vertAccel(cruise);
  m.cruise_pitch_deg = mean(cruise.map((x) => x.pitch));
  const turn = f.window('turn', 3);
  m.turn_roll_deg = mean(turn.map((x) => x.roll));
  m.turn_roll_tracking_deg = Math.max(...turn.map((x) => Math.abs(x.roll - x.rollTgt)));
  m.turn_ball_g = mean(turn.map((x) => x.acc[1]));
  m.turn_heading_rate_dps = rate(turn, 'yaw');
  m.turn_track_rate_dps = rate(turn, 'track');
  m.turn_coordination_dps = Math.abs(m.turn_heading_rate_dps - m.turn_track_rate_dps);
  m.turn_slip_deg = mean(turn.map(noseOffset));
  m.turn_vertical_accel = vertAccel(turn);
  m.headcheck_nose_offset_deg = mean(f.window('headcheck', 1).map(noseOffset));
  const hcAll = f.window('headcheck');
  m.headcheck_track_change_deg = wrap((hcAll.at(-1).track - hcAll[0].track) / DEG) * DEG;
  m.headreturn_nose_offset_deg = mean(f.window('headreturn', 0.5).map(noseOffset));
  const brake = f.window('brake');
  const stopped = brake.find((x) => x.speedH < 0.5);
  m.brake_time_to_half_mps = stopped ? stopped.t - brake[0].t : 99;
  m.brake_final_speed = brake.at(-1).speedH;
  m.thrust_to_weight_learnt = f.window('brake').at(-1).tw;
  m.mode_off_cleared = f.window('off').at(-1).active;
  return m;
}

const f1 = await sortie();
if (EXPLORE) {
  for (const s of f1.samples.filter((_, i) => i % 50 === 0)) {
    console.log([s.label.padEnd(10), s.t.toFixed(2), `spd ${s.speedH.toFixed(2)}`, `vz ${s.vel[2].toFixed(2)}`,
      `r ${s.roll.toFixed(1)}/${s.rollTgt.toFixed(1)}`, `p ${s.pitch.toFixed(1)}/${s.pitchTgt.toFixed(1)}`,
      `yaw ${s.yaw.toFixed(0)} trk ${s.track.toFixed(0)}`, `coll ${s.collective.toFixed(3)}`, `tw ${s.tw.toFixed(2)}`,
      `acc ${s.acc.map((a) => a.toFixed(2)).join(',')}`].join('  '));
  }
}
const m = measure(f1);
const f2 = await sortie();
const hash1 = await f1.hash();
m.determinism_same_hash = hash1 === (await f2.hash()) ? 1 : 0;

let failed = 0;
const rows = [];
for (const [key, value] of Object.entries(m)) {
  const band = thresholds[key];
  let verdict = 'info';
  if (band) {
    const ok = value >= band.min && value <= band.max;
    verdict = ok ? 'PASS' : 'FAIL';
    failed += ok ? 0 : 1;
  }
  rows.push({ key, value, band, verdict });
  const b = band ? `[${band.min}, ${band.max}]` : '';
  console.log(`${verdict.padEnd(4)}  ${key.padEnd(30)} ${Number(value).toFixed(3).padStart(10)}  ${b}`);
}
await mkdir(join(root, 'build/bank'), { recursive: true });
await writeFile(join(root, 'build/bank/report.json'), JSON.stringify({ rows, hash: hash1 }, null, 2));
console.log(failed ? `bank:check: ${failed} FAILED` : 'bank:check: all banded checks pass');
process.exit(failed && !EXPLORE ? 1 : 0);
