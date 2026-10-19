/*
 * bank-bench.js: bank mode against ideal acro on the three racing figures.
 *
 * Bank mode is meant to be a racing mode: as fast in a straight line, as hard
 * on a punch and as tight in a turn as acro flown perfectly. This script flies
 * each figure twice from the same start, once with bank mode's sticks and
 * once with an ideal acro pilot, and prints both with their ratio:
 *
 *   top speed   level flight, everything forward: bank full Speed, the Lift
 *               stick holding the height; acro full throttle, tilted as far
 *               as holds the height.
 *   punch       from a hover, everything up for one second: bank full Lift;
 *               acro full throttle, level.
 *   turn        from a 20 m/s cruise, a 180 degree reversal: bank full Bank
 *               with Speed at the centre and the Lift stick holding the
 *               height; acro full throttle, the
 *               thrust banked as far as holds the height, the nose on the
 *               track.
 *
 * The ideal acro pilot reads the plant's true attitude and velocity and flies
 * linear rates through a quaternion attitude loop: a pilot with perfect eyes
 * and hands, which is the bound bank mode is measured against. Bank mode
 * itself only sees what the flight controller sees. Both start from the same
 * state, set up in bank mode.
 *
 * Seconds of wall clock, no browser. Informational: no bands yet, the ratios
 * are what the owner judges the rewrite by.
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
import { composeConfig, RATES_KEEP } from '../src/fc/dump.js';
import { loadSim, SIM_OK, simErrorName } from '../tests/lib/simmod.js';
import { must } from '../tests/lib/replay.js';

const root = dirname(dirname(fileURLToPath(import.meta.url)));
const VERBOSE = process.argv.includes('--verbose');
const RX_HZ = 250;
const STEP_HZ = 1000;
const DEG = 180 / Math.PI;
/* Linear rates, so the ideal pilot's stick is its body rate over this. */
const MAX_RATE = 1000;
const LINEAR = { rcRate: 100, srate: 100, expo: 0 };
const RATES = { type: 'ACTUAL', roll: LINEAR, pitch: LINEAR, yaw: LINEAR, throttleCap: 100, thrMid: 50, thrExpo: 0 };

const wasm = await readFile(join(root, 'dist/sim-bank.wasm')).catch(() => {
  console.error('bank:bench: dist/sim-bank.wasm missing. Run npm run build:wasm:bank');
  process.exit(1);
});
const defaultDiff = await readFile(join(root, 'configs/betaflight-default.diff'), 'utf8');

/* Small vector and rotation helpers. Plant frames are z up, y left; the
 * quaternion (w, x, y, z) takes body to world. */
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const scale = (a, k) => a.map((x) => x * k);
const sub = (a, b) => a.map((x, i) => x - b[i]);
const norm = (a) => Math.hypot(...a);
const unit = (a) => scale(a, 1 / Math.max(norm(a), 1e-9));
function matOf([w, x, y, z]) {
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
  ];
}
const col = (m, j) => [m[0][j], m[1][j], m[2][j]];
function wrap(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

/* Bank stick intent to the ABI's channels, as bank-check.js. */
function bankChannels({ speed = 0, bank = 0, lift = 0.5 }) {
  return { roll: 0, pitch: -(2 * lift - 1), yaw: bank, throttle: speed };
}

/* The bank pilot holds the height with the Lift stick, as a pilot does:
 * Lift is a force, so the centre of the stick carries any vertical speed. */
const HOLD = 'hold';
const holdLift = (s) => Math.max(0, Math.min(1, 0.5 - 0.03 * s.vel[2]));

async function newFlight() {
  const sim = await loadSim(wasm);
  const code = sim.init(composeConfig(defaultDiff, RATES, RATES_KEEP));
  if (code !== SIM_OK) {
    throw new Error(`sim_init: ${simErrorName(code)}`);
  }
  must(sim.setCellVoltage(4.2), 'sim_set_cell_voltage');
  must(sim.e.sim_set_gravity(airframeById('5inch').gravityBase), 'sim_set_gravity');
  must(sim.reset(), 'sim_reset');
  must(sim.e.sim_set_ground(1, 0, 0, 1, 0, 0, 0, 0.55, 0.28), 'sim_set_ground');
  let t = 0;
  let prevVel = null;
  const telPtr = sim.e.malloc(32 * 8);
  const f = {
    sim,
    t: () => t,
    state() {
      const s = sim.readState().state;
      const vel = [...s.slice(4, 7)];
      return { t: s[0], pos: [...s.slice(1, 4)], vel, R: matOf([...s.slice(7, 11)]) };
    },
    bank(on) { must(sim.e.sim_set_bank_mode(on ? 1 : 0), 'sim_set_bank_mode'); },
    tel() {
      const n = sim.e.sim_bank_telemetry(telPtr, 32);
      return Array.from(new Float64Array(sim.e.memory.buffer, telPtr, n));
    },
    /* One RC frame with these channels. */
    frame(ch) {
      must(sim.input(t, ch.roll, ch.pitch, ch.yaw, ch.throttle), 'sim_input');
      must(sim.step(STEP_HZ / RX_HZ), 'sim_step');
      t += 1 / RX_HZ;
    },
    flyBank(seconds, sticks, each) {
      for (let i = 0; i < Math.round(seconds * RX_HZ); i += 1) {
        f.frame(bankChannels(sticks.lift === HOLD ? { ...sticks, lift: holdLift(f.state()) } : sticks));
        if (each) each(f.state());
      }
    },
    /* The ideal acro pilot: each frame `want(state)` gives a thrust direction
     * (world), a nose direction (world, horizontal) and a throttle, and the
     * attitude loop turns the craft there at up to MAX_RATE. */
    flyAcro(seconds, want, each) {
      for (let i = 0; i < Math.round(seconds * RX_HZ); i += 1) {
        const s = f.state();
        const { thrust, nose, throttle } = want(s, prevVel);
        prevVel = s.vel;
        const z = unit(thrust);
        const x = unit(sub(nose, scale(z, dot(nose, z))));
        const y = cross(z, x);
        /* Error rotation in the body frame: R^T * Rd. */
        const Rd = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]];
        const E = [0, 1, 2].map((r) => [0, 1, 2].map((c) => dot(col(s.R, r), col(Rd, c))));
        const angle = Math.acos(Math.max(-1, Math.min(1, (E[0][0] + E[1][1] + E[2][2] - 1) / 2)));
        let axis = [E[2][1] - E[1][2], E[0][2] - E[2][0], E[1][0] - E[0][1]];
        axis = norm(axis) > 1e-9 ? unit(axis) : [0, 0, 0];
        const gain = 20;    // 1/s
        const rate = scale(axis, Math.min(gain * angle, (MAX_RATE / DEG)) * DEG);
        const clamp = (v) => Math.max(-1, Math.min(1, v / MAX_RATE));
        f.frame({ roll: clamp(rate[0]), pitch: clamp(-rate[1]), yaw: clamp(-rate[2]), throttle });
        if (each) each(f.state());
      }
    },
  };
  return f;
}

/* The shared start: take off in bank mode and hover at about 20 m. */
async function hovering() {
  const f = await newFlight();
  f.bank(true);
  f.flyBank(1.0, { speed: 0, lift: 0.5 });
  for (let i = 0; i < 100 && f.state().pos[2] < 15; i += 1) {
    f.flyBank(0.1, { speed: 0, lift: 0.7 });
  }
  f.flyBank(5.0, { speed: 0, lift: HOLD });
  return f;
}

/* Cruising level at the centre of the Speed stick, about 20 m/s. */
async function cruising() {
  const f = await hovering();
  f.flyBank(8.0, { speed: 0.5, lift: HOLD });
  return f;
}

const horizontal = (v) => Math.hypot(v[0], v[1]);
const trackOf = (v) => Math.atan2(v[1], v[0]);
const UP = [0, 0, 1];

/* Ideal acro straight-line: full throttle, pitched along the travel as far as
 * keeps the height, by a slow loop on the vertical speed. */
function acroLevel(heading) {
  let tilt = 0.5;
  return (s) => {
    tilt = Math.max(0, Math.min(1.5, tilt + 0.004 * s.vel[2] * 4));
    const thrust = [Math.cos(heading) * Math.sin(tilt), Math.sin(heading) * Math.sin(tilt), Math.cos(tilt)];
    return { thrust, nose: [Math.cos(heading), Math.sin(heading), 0], throttle: 1 };
  };
}

async function topSpeed() {
  const out = {};
  {
    const f = await hovering();
    const speeds = [];
    const z0 = f.state().pos[2];
    f.flyBank(12, { speed: 1, lift: HOLD }, (s) => {
      speeds.push(horizontal(s.vel));
      if (VERBOSE && speeds.length % 50 === 0) {
        const tel = f.tel();
        console.log(`top speed bank t ${s.t.toFixed(1)} alt ${s.pos[2].toFixed(1)} vz ${s.vel[2].toFixed(1)} spd ${horizontal(s.vel).toFixed(1)} pitch ${(Math.asin(-s.R[2][0]) * DEG).toFixed(0)} tgt ${tel[2].toFixed(0)} coll ${tel[4].toFixed(2)} dbg ${tel.slice(9, 17).map((a) => a.toFixed(0)).join(',')}`);
      }
    });
    out.bank = { speed: Math.max(...speeds.slice(-RX_HZ * 2)), climb: f.state().pos[2] - z0 };
  }
  {
    const f = await hovering();
    f.bank(false);
    const speeds = [];
    const z0 = f.state().pos[2];
    f.flyAcro(12, acroLevel(0), (s) => speeds.push(horizontal(s.vel)));
    out.acro = { speed: Math.max(...speeds.slice(-RX_HZ * 2)), climb: f.state().pos[2] - z0 };
  }
  return out;
}

async function punch() {
  const out = {};
  {
    const f = await hovering();
    const z0 = f.state().pos[2];
    f.flyBank(1.0, { speed: 0, lift: 1 });
    const s = f.state();
    out.bank = { height: s.pos[2] - z0, vz: s.vel[2] };
  }
  {
    const f = await hovering();
    f.bank(false);
    const z0 = f.state().pos[2];
    f.flyAcro(1.0, () => ({ thrust: UP, nose: [1, 0, 0], throttle: 1 }));
    const s = f.state();
    out.acro = { height: s.pos[2] - z0, vz: s.vel[2] };
  }
  return out;
}

/* Track reversal: the time to turn the track 180 degrees, the widest the
 * craft gets from the entry line, the height lost and the exit speed. */
function reversal() {
  let start = null;
  let done = null;
  let wide = 0;
  return {
    each(s) {
      if (!start) {
        start = { t: s.t, pos: s.pos, track: trackOf(s.vel), speed: horizontal(s.vel) };
      }
      if (done) {
        return;
      }
      const d = sub(s.pos, start.pos);
      const across = -Math.sin(start.track) * d[0] + Math.cos(start.track) * d[1];
      wide = Math.max(wide, Math.abs(across));
      if (Math.abs(wrap(trackOf(s.vel) - start.track)) > Math.PI * 0.99 || (s.t - start.t > 0.5 && Math.abs(wrap(trackOf(s.vel) - start.track)) > Math.PI * 0.97)) {
        done = { time: s.t - start.t, width: wide, drop: start.pos[2] - s.pos[2], exit: horizontal(s.vel) };
      }
    },
    result() { return { entry: start.speed, ...(done ?? { time: 99, width: wide, drop: 0, exit: 0 }) }; },
  };
}

async function turn() {
  const out = {};
  {
    const f = await cruising();
    const r = reversal();
    let n = 0;
    f.flyBank(6, { speed: 0.5, lift: HOLD, bank: 1 }, (s) => {
      r.each(s);
      if (VERBOSE && n++ % 10 === 0 && n < 700) {
        const tel = f.tel();
        const R = s.R;
        console.log(`turn bank t ${s.t.toFixed(2)} alt ${s.pos[2].toFixed(1)} vz ${s.vel[2].toFixed(1)} spd ${horizontal(s.vel).toFixed(1)} trk ${(trackOf(s.vel) * DEG).toFixed(0)} nose ${(Math.atan2(R[1][0], R[0][0]) * DEG).toFixed(0)} up ${R[2][2].toFixed(2)} tgt r ${tel[1].toFixed(0)} p ${tel[2].toFixed(0)} yr ${tel[3].toFixed(0)} coll ${tel[4].toFixed(2)} dbg ${tel.slice(9, 17).map((a) => a.toFixed(0)).join(',')}`);
      }
    });
    out.bank = r.result();
  }
  {
    const f = await cruising();
    f.bank(false);
    const r = reversal();
    /* Banked right, as far as holds the height: the tilt rises while the
     * craft climbs and falls back while it sinks. */
    let bank = 1.2;
    f.flyAcro(6, (s) => {
      bank = Math.max(0.5, Math.min(1.5, bank + 0.004 * s.vel[2] * 4));
      const v = unit([s.vel[0], s.vel[1], 0]);
      const right = cross(v, UP);
      return { thrust: [...sub(scale(right, Math.sin(bank)), [0, 0, -Math.cos(bank)])], nose: v, throttle: 1 };
    }, r.each);
    out.acro = r.result();
  }
  return out;
}

const results = { topSpeed: await topSpeed(), punch: await punch(), turn: await turn() };
const rows = [
  ['top speed, m/s (higher)', results.topSpeed.bank.speed, results.topSpeed.acro.speed, 1],
  ['  height change, m', results.topSpeed.bank.climb, results.topSpeed.acro.climb, 0],
  ['punch, height in 1 s, m (higher)', results.punch.bank.height, results.punch.acro.height, 1],
  ['  vertical speed at 1 s, m/s', results.punch.bank.vz, results.punch.acro.vz, 0],
  ['turn 180, time, s (lower)', results.turn.bank.time, results.turn.acro.time, -1],
  ['  width, m (lower)', results.turn.bank.width, results.turn.acro.width, -1],
  ['  entry speed, m/s', results.turn.bank.entry, results.turn.acro.entry, 0],
  ['  exit speed, m/s', results.turn.bank.exit, results.turn.acro.exit, 0],
  ['  height lost, m', results.turn.bank.drop, results.turn.acro.drop, 0],
];
console.log(`${'figure'.padEnd(34)} ${'bank'.padStart(8)} ${'acro'.padStart(8)}  bank as % of acro`);
for (const [name, bank, acro, sense] of rows) {
  const pct = sense === 0 ? '' : `${(100 * (sense > 0 ? bank / acro : acro / bank)).toFixed(0)} %`;
  console.log(`${name.padEnd(34)} ${bank.toFixed(2).padStart(8)} ${acro.toFixed(2).padStart(8)}  ${pct}`);
}
await mkdir(join(root, 'build/bank'), { recursive: true });
await writeFile(join(root, 'build/bank/bench.json'), JSON.stringify(results, null, 2));
if (VERBOSE) {
  console.log(JSON.stringify(results, null, 2));
}
