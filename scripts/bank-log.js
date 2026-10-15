/*
 * bank-log.js: read a bank flight log, and fly its sticks again headless.
 *
 * The page records every bank flight to testing-flight-logs/auto/ (see
 * BANK-MODE.md, "Flight logs, automatically"). This turns such a file into
 * something to act on:
 *
 *   npm run bank:log                       summary of the newest log, half a second per row
 *   npm run bank:log -- FILE               the same for a given file
 *   npm run bank:log -- --rows 12 14 [N]   every Nth row (default 5) between 12 s and 14 s
 *   npm run bank:log -- --replay           fly the log's sticks through dist/sim-bank.wasm and
 *                                          summarise that flight the same way, beside the log's
 *
 * READING THE SUMMARY. Each row is half a second while Bank is on. "gyroP"
 * is the span and the number of direction changes of the pitch rate, the
 * plainest sign of an oscillation. "pTgt" is how far the firmware's own pitch
 * target moved: a pitch rate that swings while the target stays put is the
 * attitude loop or the plant; one that swings with the target is the
 * firmware commanding it. The debug columns (DEBUG_BANK, see the DEBUG_SET
 * calls in the Betaflight branch's flight/bank_mode.c) say which estimate
 * moved: speed estimate, thrust trim, thrust to weight.
 *
 * THE REPLAY flies the logged sticks at the logged rows' times, switching Bank
 * on and off where the log's "bank active" column does, at the five inch's
 * gravity from the ground, as scripts/bank-check.js flies. It will not follow
 * the logged flight exactly (the page started differently, and small
 * differences grow), but it reproduces what the sticks provoke, and it runs
 * whatever firmware dist/sim-bank.wasm was last built from, so a fix can be
 * tried against the flight that showed the problem.
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

import { readdir, readFile, stat } from 'node:fs/promises';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

import { airframeById } from '../configs/airframes.js';
import { RATE_DEFAULTS } from '../configs/rates.js';
import { composeConfig, RATES_KEEP } from '../src/fc/dump.js';
import { loadSim, SIM_OK, simErrorName } from '../tests/lib/simmod.js';
import { must } from '../tests/lib/replay.js';

const root = dirname(dirname(fileURLToPath(import.meta.url)));
const RX_HZ = 250;
const STEP_HZ = 1000;
const DEG = 180 / Math.PI;
const WINDOW_S = 0.5;

const args = process.argv.slice(2);
const flag = (name) => args.indexOf(name);

async function newestLog() {
  for (const dir of ['testing-flight-logs/auto', 'testing-flight-logs']) {
    const names = (await readdir(join(root, dir)).catch(() => [])).filter((n) => n.endsWith('.csv'));
    const dated = await Promise.all(names.map(async (n) => ({ n: join(root, dir, n), t: (await stat(join(root, dir, n))).mtimeMs })));
    if (dated.length > 0) {
      return dated.sort((a, b) => b.t - a.t)[0].n;
    }
  }
  throw new Error('no flight log found in testing-flight-logs/auto or testing-flight-logs');
}

/* One row per CSV line, by column name, so an older log without the bank or
 * debug columns still reads (those fields come back undefined). */
function parseLog(text) {
  const lines = text.trim().split('\n');
  const head = lines[0].split(',').map((h) => h.trim().replace(/"/g, ''));
  const col = (name) => head.indexOf(name);
  const at = (r, name) => (col(name) >= 0 ? r[col(name)] : undefined);
  return lines.slice(1).map((l) => l.split(',').map(Number)).filter((r) => r.length === head.length).map((r) => ({
    t: r[0] / 1e6,
    /* blackbox stick units back to the ABI's: see src/share/flightlog.js */
    head: at(r, 'rcCommand[0]') / 500,
    lift: 0.5 + at(r, 'rcCommand[1]') / 1000,
    bank: at(r, 'rcCommand[2]') / 500,
    speed: (at(r, 'rcCommand[3]') - 1000) / 1000,
    gyroP: at(r, 'gyroADC[1]'),
    active: at(r, 'bank active') ?? 1,
    pTgt: at(r, 'bank pitch target (deg)'),
    rTgt: at(r, 'bank roll target (deg)'),
    coll: at(r, 'bank collective'),
    pitch: at(r, 'pitch (deg)'),
    alt: at(r, 'altitude (m)'),
    vz: at(r, 'vz (m/s)'),
    spd: at(r, 'speed (m/s)'),
    dbg: [0, 1, 2, 3, 4, 5, 6, 7].map((i) => at(r, `debug[${i}]`)),
  }));
}

const mean = (a) => a.reduce((s, x) => s + x, 0) / Math.max(1, a.length);
const span = (a) => (a.length ? Math.max(...a) - Math.min(...a) : 0);
const fmt = (x, d = 1) => (x === undefined || Number.isNaN(x) ? '-' : x.toFixed(d));

function summarise(rows, title) {
  console.log(`\n${title}`);
  console.log('   t   | gyroP span dirs | pTgt span | coll      | lift  speed | vz    alt  spd  | speedEst  trim  T/W');
  const end = rows.at(-1).t;
  for (let s = Math.floor(rows[0].t / WINDOW_S) * WINDOW_S; s < end; s += WINDOW_S) {
    const w = rows.filter((r) => r.t >= s && r.t < s + WINDOW_S);
    if (w.length < 3 || mean(w.map((r) => r.active)) < 0.5) {
      continue;
    }
    let dirs = 0;
    for (let i = 2; i < w.length; i += 1) {
      if (Math.sign(w[i].gyroP - w[i - 1].gyroP) !== Math.sign(w[i - 1].gyroP - w[i - 2].gyroP)) {
        dirs += 1;
      }
    }
    const gspan = span(w.map((r) => r.gyroP));
    const tspan = w[0].pTgt === undefined ? undefined : span(w.map((r) => r.pTgt));
    const mark = gspan > 150 && tspan !== undefined && tspan < 10 ? '  << pitch swings, target steady' : gspan > 150 ? '  << pitch swings' : '';
    const last = w.at(-1);
    const d = (i, scale) => (last.dbg[i] === undefined ? '-' : (last.dbg[i] / scale).toFixed(2));
    console.log(`${fmt(s).padStart(6)} | ${fmt(gspan, 0).padStart(9)} ${String(dirs).padStart(5)} | ${fmt(tspan).padStart(9)} | `
      + `${fmt(Math.min(...w.map((r) => r.coll)), 2)}-${fmt(Math.max(...w.map((r) => r.coll)), 2)} | `
      + `${fmt(mean(w.map((r) => r.lift)), 2)}  ${fmt(mean(w.map((r) => r.speed)), 2)}  | `
      + `${fmt(last.vz).padStart(5)} ${fmt(last.alt, 0).padStart(4)} ${fmt(last.spd).padStart(4)} | `
      + `${d(0, 1000).padStart(8)} ${d(2, 1000).padStart(5)} ${d(4, 100).padStart(5)}${mark}`);
  }
}

function dumpRows(rows, from, to, every) {
  for (const [i, r] of rows.entries()) {
    if (r.t >= from && r.t <= to && i % every === 0) {
      console.log(`${r.t.toFixed(3)} act ${r.active} lift ${fmt(r.lift, 2)} speed ${fmt(r.speed, 2)} bank ${fmt(r.bank, 2)} head ${fmt(r.head, 2)}`
        + ` | gyroP ${fmt(r.gyroP, 0)} pitch ${fmt(r.pitch)}/${fmt(r.pTgt)} roll tgt ${fmt(r.rTgt)} | coll ${fmt(r.coll, 3)}`
        + ` | alt ${fmt(r.alt)} vz ${fmt(r.vz)} spd ${fmt(r.spd)} | debug ${r.dbg.map((x) => fmt(x, 0)).join(' ')}`);
    }
  }
}

/* The scripts/bank-check.js setup: five inch gravity, from the ground. */
async function replay(rows) {
  const wasm = await readFile(join(root, 'dist/sim-bank.wasm')).catch(() => {
    throw new Error('dist/sim-bank.wasm missing. Run npm run build:wasm:bank');
  });
  const sim = await loadSim(wasm);
  const code = sim.init(composeConfig(await readFile(join(root, 'configs/betaflight-default.diff'), 'utf8'), RATE_DEFAULTS, RATES_KEEP));
  if (code !== SIM_OK) {
    throw new Error(`sim_init: ${simErrorName(code)}`);
  }
  must(sim.setCellVoltage(4.0), 'sim_set_cell_voltage');
  must(sim.e.sim_set_gravity(airframeById('5inch').gravityBase), 'sim_set_gravity');
  must(sim.reset(), 'sim_reset');
  must(sim.e.sim_set_ground(1, 0, 0, 1, 0, 0, 0, 0.55, 0.28), 'sim_set_ground');
  const telPtr = sim.e.malloc(32 * 8);
  const out = [];
  let k = 0;
  let bankOn = false;
  let prevPitch = 0;
  const t0 = rows[0].t;
  for (let t = 0; t < rows.at(-1).t - t0; t += 1 / RX_HZ) {
    while (k + 1 < rows.length && rows[k + 1].t - t0 <= t) {
      k += 1;
    }
    const r = rows[k];
    if ((r.active >= 0.5) !== bankOn) {
      bankOn = r.active >= 0.5;
      must(sim.e.sim_set_bank_mode(bankOn ? 1 : 0), 'sim_set_bank_mode');
    }
    must(sim.input(t, r.head, -(2 * r.lift - 1), r.bank, r.speed), 'sim_input');
    must(sim.step(STEP_HZ / RX_HZ), 'sim_step');
    const { state } = sim.readState();
    const n = sim.e.sim_bank_telemetry(telPtr, 32);
    const tel = Array.from(new Float64Array(sim.e.memory.buffer, telPtr, n));
    const [w, x, y, z] = state.slice(7, 11);
    const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))) * DEG;
    out.push({
      t: t + t0, head: r.head, lift: r.lift, bank: r.bank, speed: r.speed,
      gyroP: (pitch - prevPitch) * RX_HZ, active: tel[0], pTgt: tel[2], rTgt: tel[1], coll: tel[4], pitch,
      alt: state[3], vz: state[6], spd: Math.hypot(state[4], state[5]), dbg: tel.slice(9, 17),
    });
    prevPitch = pitch;
  }
  return out;
}

const fileArg = args.find((a, i) => a.endsWith('.csv') && !['--rows'].includes(args[i - 1]));
const file = fileArg ?? await newestLog();
const rows = parseLog(await readFile(file, 'utf8'));
console.log(`${file}: ${rows.length} rows over ${fmt(rows.at(-1).t - rows[0].t)} s, about ${fmt(rows.length / (rows.at(-1).t - rows[0].t), 0)} rows/s`);
if (flag('--rows') >= 0) {
  const i = flag('--rows');
  dumpRows(rows, Number(args[i + 1] ?? 0), Number(args[i + 2] ?? 1e9), Number(args[i + 3] ?? 5));
} else {
  summarise(rows, 'LOG (Bank on, half second per row)');
  if (flag('--replay') >= 0) {
    summarise(await replay(rows), 'REPLAY through dist/sim-bank.wasm (gyroP is from the attitude, 250 Hz)');
  }
}
