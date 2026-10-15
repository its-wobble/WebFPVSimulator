/*
 * flightlog.js: record a flight and write it out as a blackbox log.
 *
 * WHY THE SIMULATOR RECORDS. scripts/replay-log.js can take a real quad's
 * blackbox log, fly its sticks through this build and report what the plant
 * got wrong. That is the instrument. This is the other half of it: the
 * simulator writing its OWN flight in the same format, for two jobs that
 * both matter.
 *
 *   Diagnosis. "It feels different" is not something a maintainer can act
 *   on, and it cost this project a whole archaeology session once already.
 *   A log of the flight that felt wrong is: rates, sticks, pack voltage and
 *   rotor speeds against time, which either shows the anomaly or rules the
 *   plant out.
 *
 *   Comparison. A sim log and a real log in the same format go through the
 *   same parser and the same report, so "here is my quad, here is the sim,
 *   flying the same line" is a diff rather than an argument.
 *
 * THE FORMAT is blackbox_decode's CSV, not Betaflight's packed binary. The
 * binary is a format this project does not own and reimplementing its
 * writer would be a second decoder to keep in sync for no gain: every tool
 * that reads blackbox reads the decoded CSV too, and tests/lib/blackbox.js
 * parses exactly this.
 *
 * WHAT IS AND IS NOT IN A ROW. Time, sticks, gyro and pack voltage are the
 * real thing, straight off the state block. The motor columns carry ROTOR
 * SPEED normalised against a nominal full throttle, NOT the duty a real ESC
 * logs, because duty never crosses the ABI: the module reports RPM. That is
 * written down here rather than left for someone to discover, because a
 * motor column that looks like a duty and is not is exactly the sort of
 * thing that quietly ruins a comparison.
 *
 * RATE. One row per rendered frame, so 60 to 144 Hz rather than a real
 * FC's 1 to 8 kHz. The stick and gyro pair in a row are read at the same
 * instant, so the row is honest; there are simply fewer of them. Enough to
 * see a manoeuvre, not enough to see filter phase.
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

/* Betaflight stick units, the same ones tests/lib/blackbox.js inverts. */
const RC_SPAN = 500;
const THR_MIN = 1000;
const THR_SPAN = 1000;

/*
 * Write rows as blackbox_decode's CSV.
 *
 * Lives here, in src, rather than in tests, because the browser is what
 * produces a log; tests/lib/blackbox.js imports this for the round trip
 * that proves its own parser agrees with this writer.
 *
 * The pitch channel is negated on the way out for the same reason the
 * parser negates it on the way in: bf_glue.c builds rcData[PITCH] as
 * 1500 MINUS the ABI channel, so a log's positive pitch is this ABI's
 * negative pitch. The two negations are one convention, and they are
 * tested against each other by scripts/replay-log.js --selftest.
 */
/*
 * Bank mode columns, written only by a bank build (dist/sim-bank.wasm). The
 * first nine are the firmware's own outputs and inputs (sim_bank_telemetry),
 * the rest are plant truth for the reader: what the craft really did. Names
 * are not blackbox's, so a blackbox reader, which picks columns by name,
 * skips them. The firmware never sees the plant truth; the log is a
 * diagnostic, like the bank check.
 */
export const BANK_COLUMNS = [
  'bank active', 'bank roll target (deg)', 'bank pitch target (deg)', 'bank yaw rate (deg/s)',
  'bank collective', 'bank thrust to weight', 'bank acc x (g)', 'bank acc y (g)', 'bank acc z (g)',
  'roll (deg)', 'pitch (deg)', 'yaw (deg)', 'altitude (m)', 'vz (m/s)', 'speed (m/s)',
  /* DEBUG_BANK, the mode's own estimates, under blackbox's own names: speed
   * and target speed (0.001 g), vertical speed (0.01 m/s), vertical damping
   * (0.001 g), thrust to weight (0.01), turn gain (0.1 deg/s g), ball and the
   * vertical speed's drag correction (0.001 g). See bank_mode.c. */
  'debug[0]', 'debug[1]', 'debug[2]', 'debug[3]', 'debug[4]', 'debug[5]', 'debug[6]', 'debug[7]',
];

export function toBlackboxCsv(samples, { header = true, bank = samples.some((s) => s.bank) } = {}) {
  const head = [
    'time (us)',
    'rcCommand[0]', 'rcCommand[1]', 'rcCommand[2]', 'rcCommand[3]',
    'gyroADC[0]', 'gyroADC[1]', 'gyroADC[2]',
    'motor[0]', 'motor[1]', 'motor[2]', 'motor[3]',
    'vbatLatest (V)',
    ...(bank ? BANK_COLUMNS : []),
  ].map((n) => `"${n}"`).join(', ');
  const body = samples.map((s) => [
    Math.round(s.tUs),
    s.rc[0] * RC_SPAN,
    -s.rc[1] * RC_SPAN,
    s.rc[2] * RC_SPAN,
    THR_MIN + s.rc[3] * THR_SPAN,
    s.gyroDps[0], s.gyroDps[1], s.gyroDps[2],
    ...(s.motor ?? [0, 0, 0, 0]).map((d) => d * 2047),
    s.vbat ?? 0,
    ...(bank ? (s.bank ?? new Array(BANK_COLUMNS.length).fill(0)) : []),
  ].join(', '));
  const rows = body.length > 0 ? `${body.join('\n')}\n` : '';
  return header ? `${head}\n${rows}` : rows;
}

/* Roll, pitch and yaw in degrees from the state block's quaternion (w x y z),
 * Z up, the same convention scripts/bank-check.js reads. Display only. */
function eulerDeg(w, x, y, z) {
  const d = 57.29577951308232;
  return [
    Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * d,
    Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))) * d,
    Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * d,
  ];
}

/*
 * How many rows to keep. At 144 Hz this is a little over eleven minutes,
 * which is longer than any pack, and it is a RING: a pilot who leaves
 * recording on all session still gets the end of the flight rather than an
 * out of memory. Roughly 13 MB of CSV at the cap.
 */
const MAX_ROWS = 100000;

export class FlightRecorder {
  constructor() {
    this.rows = [];
    this.on = false;
    this.dropped = 0;
    /*
     * The splice offset. The module's clock restarts at zero on every
     * reset, and a crash recovery resets the module, so a session's second
     * run used to make the file's time axis jump BACKWARDS mid stream. The
     * owner's first real log did exactly that, twice, and any tool that
     * bins by time reads such a file as one garbled flight. When a pushed
     * time is behind the last one, the offset advances so the new run
     * continues the axis after a visible 100 ms seam: one file, several
     * runs, monotonic time, and the seam wide enough that a reader can see
     * where the splice is.
     */
    this.offsetUs = 0;
    this.lastRawUs = -Infinity;
    this.lastOutUs = -Infinity;
    /* Rows ever pushed since the last clear, ring drops included, so a
     * reader that takes the log in pieces (the dev server upload) can ask
     * for what is new since its last piece. */
    this.pushed = 0;
  }

  setEnabled(on) {
    this.on = Boolean(on);
    if (!this.on) {
      return;
    }
    /* Turning it on starts a new log. A recording that silently continued
     * across runs would put two flights in one file with a time axis that
     * jumps backwards. */
    this.clear();
  }

  clear() {
    this.rows.length = 0;
    this.dropped = 0;
    this.offsetUs = 0;
    this.lastRawUs = -Infinity;
    this.lastOutUs = -Infinity;
    this.pushed = 0;
  }

  get count() {
    return this.rows.length;
  }

  /* Roughly, for a menu note. */
  get seconds() {
    if (this.rows.length < 2) {
      return 0;
    }
    return (this.rows[this.rows.length - 1].tUs - this.rows[0].tUs) / 1e6;
  }

  /*
   * One row, from the state block and the sticks that produced it.
   *
   * `st` is the raw state array and `rc` the channel set last handed to the
   * module, so the pair is what the craft was actually doing and what it
   * was actually told, read at the same instant. `bank` is the bank build's
   * telemetry (sim_bank_telemetry, nine values) or null on any other build.
   */
  push(st, rc, fullThrottleRpm, bank = null) {
    if (!this.on) {
      return;
    }
    if (this.rows.length >= MAX_ROWS) {
      this.rows.shift();
      this.dropped += 1;
    }
    const rpmToDuty = (rpm) => (fullThrottleRpm > 0 ? rpm / fullThrottleRpm : 0);
    const rawUs = st[0] * 1e6;
    if (rawUs < this.lastRawUs) {
      this.offsetUs = this.lastOutUs + 100000 - rawUs;
    }
    this.lastRawUs = rawUs;
    this.lastOutUs = rawUs + this.offsetUs;
    this.rows.push({
      tUs: this.lastOutUs,
      rc: [rc.roll, rc.pitch, rc.yaw, rc.throttle],
      /* State block P, Q, R are rad/s; a blackbox gyro column is deg/s. */
      gyroDps: [st[11] * 57.29577951308232, st[12] * 57.29577951308232, st[13] * 57.29577951308232],
      /* Rotor speed as a fraction of nominal full throttle. NOT ESC duty:
       * see the header. */
      motor: [rpmToDuty(st[14]), rpmToDuty(st[15]), rpmToDuty(st[16]), rpmToDuty(st[17])],
      vbat: st[18],
      bank: bank ? [...bank.slice(0, 9), ...eulerDeg(st[7], st[8], st[9], st[10]), st[3], st[6], Math.hypot(st[4], st[5]),
        ...Array.from({ length: 8 }, (_, i) => bank[9 + i] ?? 0)] : undefined,
    });
    this.pushed += 1;
  }

  csv() {
    return toBlackboxCsv(this.rows);
  }

  /* The rows pushed after the first `since` of them, as CSV with or without
   * the header, and the count to pass next time. Rows the ring already
   * dropped are gone, so a reader that falls 100000 rows behind gets a gap. */
  csvSince(since, header) {
    const fresh = Math.min(this.pushed - since, this.rows.length);
    const rows = fresh > 0 ? this.rows.slice(this.rows.length - fresh) : [];
    return { text: toBlackboxCsv(rows, { header, bank: this.rows.some((r) => r.bank) }), next: this.pushed };
  }
}

/*
 * Hand the file to the browser.
 *
 * An anchor with a download attribute and an object URL, which is the one
 * mechanism that works in every browser this runs in without a server
 * round trip. The URL is revoked on the next tick rather than immediately
 * because Safari has historically cancelled the download if it is revoked
 * inside the same task.
 */
export function downloadText(filename, text, mime = 'text/csv') {
  const blob = new Blob([text], { type: `${mime};charset=utf-8` });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.style.display = 'none';
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 0);
}

/* A name that sorts and says what it is. */
export function flightLogName(mapId) {
  const t = new Date();
  const p = (n, w = 2) => String(n).padStart(w, '0');
  const stamp = `${t.getFullYear()}${p(t.getMonth() + 1)}${p(t.getDate())}`
    + `-${p(t.getHours())}${p(t.getMinutes())}${p(t.getSeconds())}`;
  return `webfpv-${String(mapId || 'flight')}-${stamp}.csv`;
}
