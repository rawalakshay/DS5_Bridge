import { spawnSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import path from 'node:path';

// The AudioHelper is Windows-only (WASAPI + WinUSB), so on macOS/Linux the SDK is
// usually absent and building it would be pointless anyway. Skip instead of failing,
// so `npm run dev` still gets to the Electron app. Set DS5_BRIDGE_REQUIRE_DOTNET=1 to
// force the original hard failure (e.g. cross-publishing from a non-Windows machine).
const requireDotnet =
  process.platform === 'win32' || process.env.DS5_BRIDGE_REQUIRE_DOTNET === '1';

const executableName = process.platform === 'win32' ? 'dotnet.exe' : 'dotnet';
const configuredRoots = [
  process.env.DOTNET_ROOT,
  process.env.DOTNET_ROOT_X64,
  process.env.DOTNET_ROOT_X86
].filter((value) => typeof value === 'string' && value.trim().length > 0);

const candidates = configuredRoots
  .map((root) => path.join(root, executableName))
  .filter((candidate) => existsSync(candidate));
candidates.push('dotnet');

const uniqueCandidates = [...new Map(candidates.map((candidate) => [
  process.platform === 'win32' ? candidate.toLowerCase() : candidate,
  candidate
])).values()];

let selected = null;
for (const candidate of uniqueCandidates) {
  const probe = spawnSync(candidate, ['--list-sdks'], {
    encoding: 'utf8',
    windowsHide: true
  });
  if (probe.status === 0 && probe.stdout.trim().length > 0) {
    selected = candidate;
    break;
  }
}

if (!selected) {
  if (!requireDotnet) {
    console.warn(
      `Skipping .NET step on ${process.platform}: no SDK found, and the AudioHelper is Windows-only. ` +
        'Set DS5_BRIDGE_REQUIRE_DOTNET=1 to make this a hard failure.'
    );
    process.exit(0);
  }
  console.error(
    'No .NET SDK was found. Install an SDK or set DOTNET_ROOT to a directory containing an SDK-enabled dotnet executable.'
  );
  process.exit(1);
}

const result = spawnSync(selected, process.argv.slice(2), {
  stdio: 'inherit',
  windowsHide: true
});

if (result.error) {
  console.error(`Unable to launch ${selected}: ${result.error.message}`);
  process.exit(1);
}

process.exit(result.status ?? 1);
