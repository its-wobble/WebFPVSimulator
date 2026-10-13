/*
 * fc-catalog-gen.js: write src/fc/catalog-data.js from the 4.5.1 valueTable
 * and the keys bf_settings.c actually writes.
 *
 * Do not edit catalog-data.js by hand. Change bf_settings.c or the status
 * rules in src/fc/catalog.js, then run this. lint:catalog fails if the
 * committed data lags the firmware table.
 *
 * This file is part of WebFPVSimulator.
 *
 * WebFPVSimulator is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at
 * your option) any later version.
 *
 * WebFPVSimulator is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WebFPVSimulator. If not, see <https://www.gnu.org/licenses/>.
 */

import { writeFile } from 'node:fs/promises';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

import { loadFirmwareTables } from './fc-valuetable.js';

const root = dirname(dirname(fileURLToPath(import.meta.url)));

const { table, live } = await loadFirmwareTables(root);
const liveSet = new Set(live);

const rows = table.map((row) => ({
  key: row.key,
  type: row.type,
  lookup: row.lookup,
  pg: row.pg,
  min: row.min,
  max: row.max,
  array: row.array,
  live: liveSet.has(row.key),
}));

/*
 * Keys bf_settings.c applies that Betaflight's own valueTable does not
 * carry. There is exactly one family of them, the rpm_filter_weights, which
 * patches/0001 adds, and they really do belong to RPM_FILTER_CONFIG.
 *
 * The loop below stamps that pg on EVERY key that falls through here, so a
 * key added to bf_settings.c with a typo, or one whose valueTable entry
 * moves, would be filed silently under a parameter group it has nothing to
 * do with and the catalog would look complete. Fail instead: a new family
 * here is a decision, not a default.
 */
const extra = live.filter((k) => !table.some((r) => r.key === k));
/*
 * Keys from the banked-mode-init fork, not in the 4.5.1 valueTable. Type
 * and limits are the fork's own cli/settings.c rows, so the Configurator
 * style fields clamp the same way the fork's CLI does.
 */
const FORK_ONLY = {
  bank_thrust_to_weight: ['UINT16', '110', '1000'],
  bank_max_bank_angle: ['UINT8', '5', '80'],
  bank_max_tilt_angle: ['UINT8', '5', '60'],
  bank_stick_deadband: ['UINT8', '0', '25'],
  bank_head_check_range: ['UINT8', '0', '180'],
  bank_head_check_rate: ['UINT16', '30', '1000'],
  bank_ball_kp: ['UINT16', '0', '2000'],
  bank_ball_ki: ['UINT16', '0', '2000'],
  bank_ball_deadband: ['UINT16', '0', '500'],
  bank_yaw_max_rate: ['UINT16', '30', '1000'],
  bank_speed_kp: ['UINT16', '0', '1000'],
  bank_speed_deadband: ['UINT16', '0', '500'],
};
const unexpected = extra.filter((k) => !/^rpm_filter_weights_[123]$/.test(k) && !FORK_ONLY[k]);
if (unexpected.length) {
  throw new Error(
    `fc-catalog-gen: ${unexpected.length} key(s) in bf_settings.c are not in the `
    + `valueTable and are not rpm_filter_weights: ${unexpected.join(', ')}. `
    + 'Give each one its real parameter group here rather than letting it '
    + 'inherit RPM_FILTER_CONFIG.',
  );
}
for (const key of extra) {
  const fork = FORK_ONLY[key];
  rows.push({
    key,
    type: fork ? fork[0] : 'UINT8',
    lookup: null,
    pg: fork ? 'BANK_MODE_CONFIG' : 'RPM_FILTER_CONFIG',
    min: fork ? fork[1] : null,
    max: fork ? fork[2] : null,
    array: false,
    live: true,
  });
}

const header = `/*
 * catalog-data.js: generated from vendor/betaflight 4.5.1 valueTable and
 * src/native/bf/bf_settings.c. Do not edit. Run:
 *   node scripts/fc-catalog-gen.js
 *
 * This file is part of WebFPVSimulator.
 *
 * WebFPVSimulator is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at
 * your option) any later version.
 *
 * WebFPVSimulator is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WebFPVSimulator. If not, see <https://www.gnu.org/licenses/>.
 */

`;

const body =
  header +
  `export const VALUE_TABLE = ${JSON.stringify(rows, null, 2)};\n`;

const out = join(root, 'src/fc/catalog-data.js');
await writeFile(out, body, 'utf8');
const liveN = rows.filter((r) => r.live).length;
console.log(
  `fc-catalog-gen: wrote ${rows.length} keys (${table.length} valueTable, ${extra.length} bf_settings extras), ${liveN} live, to src/fc/catalog-data.js`,
);
