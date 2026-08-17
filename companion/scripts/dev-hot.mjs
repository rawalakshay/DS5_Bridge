// Hot-reload dev loop: Vite dev server for the renderer + Electron pointed at it.
// The renderer hot-updates on save; main/preload changes still need a restart (Ctrl+C, rerun).
import { spawn } from 'node:child_process';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';
import { createServer } from 'vite';
import electronPath from 'electron';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');

const server = await createServer({ configFile: path.join(root, 'vite.config.ts'), root });
await server.listen();
server.printUrls();

const url = server.resolvedUrls?.local?.[0];
if (!url) {
  await server.close();
  throw new Error('Vite dev server did not report a local URL.');
}

const electron = spawn(electronPath, ['.'], {
  cwd: root,
  stdio: 'inherit',
  env: { ...process.env, DS5_BRIDGE_DEV_SERVER_URL: url }
});

let shuttingDown = false;
const shutdown = async (code) => {
  if (shuttingDown) return;
  shuttingDown = true;
  await server.close();
  process.exit(code ?? 0);
};

electron.on('close', (code) => void shutdown(code ?? 0));
process.on('SIGINT', () => {
  electron.kill('SIGINT');
  void shutdown(0);
});
process.on('SIGTERM', () => {
  electron.kill('SIGTERM');
  void shutdown(0);
});
