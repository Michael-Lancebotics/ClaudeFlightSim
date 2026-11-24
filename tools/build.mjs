// Builds the browser app.
//   node tools/build.mjs          -> dist/ and the single-file ClaudeFlightSim.html
//   node tools/build.mjs --serve  -> local dev server on http://localhost:8000 with rebuilds
import * as esbuild from 'esbuild';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const serve = process.argv.includes('--serve');

const options = {
  entryPoints: [join(root, 'src/app/main.js')],
  bundle: true,
  minify: !serve,
  sourcemap: serve ? 'inline' : false,
  format: 'iife',
  target: ['chrome100', 'edge100', 'firefox100'],
  write: false,
  outdir: join(root, 'dist'),
  legalComments: 'none',
  logLevel: 'info',
};

function writeOutputs(result) {
  let js = '', css = '';
  for (const f of result.outputFiles) {
    if (f.path.endsWith('.js')) js = f.text;
    if (f.path.endsWith('.css')) css = f.text;
  }
  const tpl = readFileSync(join(root, 'index.html'), 'utf8');
  mkdirSync(join(root, 'dist'), { recursive: true });
  // dist/: separate files (for a web server)
  writeFileSync(join(root, 'dist/app.js'), js);
  writeFileSync(join(root, 'dist/app.css'), css);
  writeFileSync(join(root, 'dist/index.html'),
    tpl.replace('<!--CSS-->', '<link rel="stylesheet" href="app.css">').replace('<!--JS-->', '<script src="app.js"></script>'));
  // single self-contained file that works when double-clicked (file://)
  const safeJs = js.replace(/<\/script/gi, '<\\/script');
  const single = tpl.replace('<!--CSS-->', () => `<style>\n${css}\n</style>`).replace('<!--JS-->', () => `<script>\n${safeJs}\n</script>`);
  writeFileSync(join(root, 'ClaudeFlightSim.html'), single);
  console.log(`wrote ClaudeFlightSim.html (${(single.length / 1024).toFixed(0)} KB) and dist/`);
}

if (serve) {
  const ctx = await esbuild.context({
    ...options,
    plugins: [{ name: 'out', setup(b) { b.onEnd((r) => { if (r.outputFiles) writeOutputs(r); }); } }],
  });
  await ctx.watch();
  const { port } = await ctx.serve({ servedir: join(root, 'dist'), port: 8000 });
  console.log(`serving on http://localhost:${port}`);
} else {
  const result = await esbuild.build(options);
  writeOutputs(result);
}
