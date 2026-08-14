/**
 * Writes the built-in sample robots from assets/js/samples.js out to samples/*.urdf
 * so they can be downloaded and drag-and-dropped into the tool.
 *
 *   node tools/export-samples.mjs
 */
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');

const global = {};
new Function('window', readFileSync(resolve(root, 'assets/js/samples.js'), 'utf8'))(global);

mkdirSync(resolve(root, 'samples'), { recursive: true });
for (const sample of global.SAMPLES) {
  const file = resolve(root, 'samples', `${sample.id}.urdf`);
  writeFileSync(file, sample.text, 'utf8');
  console.log(`wrote samples/${sample.id}.urdf  (${sample.label.en})`);
}
