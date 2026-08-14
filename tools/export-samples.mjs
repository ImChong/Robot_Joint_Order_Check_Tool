/**
 * Writes the synthetic sample robots from assets/js/samples.js out to samples/
 * so they can be downloaded and drag-and-dropped into the tool.
 *
 * Samples without inline `text` are vendored third-party files (see
 * samples/README.md); those are only checked for existence, never rewritten.
 *
 *   node tools/export-samples.mjs
 */
import { readFileSync, writeFileSync, mkdirSync, existsSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');

const global = {};
new Function('window', readFileSync(resolve(root, 'assets/js/samples.js'), 'utf8'))(global);

mkdirSync(resolve(root, 'samples'), { recursive: true });

let missing = 0;
for (const sample of global.SAMPLES) {
  const name = sample.file || `${sample.id}.urdf`;
  const file = resolve(root, 'samples', name);

  if (!sample.text) {
    if (existsSync(file)) {
      console.log(`kept   samples/${name}  (vendored — ${sample.label.en})`);
    } else {
      console.error(`MISSING samples/${name}  (vendored — ${sample.label.en})`);
      missing++;
    }
    continue;
  }

  writeFileSync(file, sample.text, 'utf8');
  console.log(`wrote  samples/${name}  (${sample.label.en})`);
}

if (missing) {
  console.error(`${missing} vendored sample file(s) missing — the sample buttons for them will fail.`);
  process.exit(1);
}
