# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Two coupled deliverables that ship as one release:

1. **Firmware** (`src/`, C/C++20) for a Raspberry Pi Pico 2 W / Waveshare RP2350B-Plus-W. It pairs with a real DualSense over Bluetooth (BTstack) and presents itself to a Windows host as a USB DualSense-compatible device (TinyUSB), bridging input, audio, haptics, and triggers in both directions.
2. **Companion app** (`companion/`, Electron + React + TypeScript) that configures the running bridge over a vendor HID/WinUSB protocol, plus a self-contained .NET 9 `AudioHelper` (`companion/native/AudioHelper/`) that owns all Windows audio/WinUSB work.

The companion app and its build/packaging are **Windows-only**. On macOS/Linux only the firmware cross-build and the firmware C++ tests are runnable.

## Commands

### Firmware

```bash
git submodule update --init --recursive   # lib/opus, lib/WDL

cmake -S . -B build/companion -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DENABLE_COMPANION=ON
cmake --build build/companion --target ds5-bridge   # -> build/companion/ds5-bridge.uf2
```

- `-DENABLE_COMPANION=ON` is required for the companion app to see the bridge; it is **OFF** by default.
- The repo-root `Makefile` wraps the whole flow: `make pico-build` drops a timestamped image into `uf2_builds/`.
- Waveshare board: add `-DWAVESHARE_RP2350B_PLUS_W_BUILD=ON`, or run `boards/build_waveshare_rp2350b_plus_w.sh`.
- Toolchain must match CI: Pico SDK `2.3.0`, TinyUSB `2d56dc533e45e4e91b15e93fdab5e22e964f328d`, Arm GNU `15.2.Rel1`. The Ubuntu apt `gcc-arm-none-eabi` 13.2 produces audible audio static on Waveshare builds — use ARM's official tarball.
- Diagnostics are compile-time: `-DDS5_DIAGNOSTICS_PRESET=off|audio|traces|all|custom`. Presets are authoritative and override stale cached legacy flags. UART logging preset: `cmake --preset pico2-w-debug-uart-companion-on`. See `docs/diagnostics.md`.

### Firmware tests (host build, no Pico SDK needed)

```bash
cmake -S tests/firmware -B build-firmware-tests -G Ninja
cmake --build build-firmware-tests
ctest --test-dir build-firmware-tests --output-on-failure
ctest --test-dir build-firmware-tests -R usb_descriptor_migration_test --output-on-failure   # single test
```

### Companion app (Windows)

```bash
cd companion
npm ci
npm run typecheck        # tsc for main+preload+shared, then renderer+shared
npm test                 # companion vitest + installer check + .NET tests + firmware ctest
npm run dev              # build then launch electron
npm run build            # audio helper + app
npm run package:win      # unpacked
npm run installer:win    # NSIS installer
```

Single tests:

```bash
npx vitest run src/main/bridge-service.test.ts
npx vitest run src/shared/protocol.test.ts -t "some test name"
node scripts/run-dotnet-sdk.mjs test native/AudioHelper.Tests/AudioHelper.Tests.csproj --filter FullyQualifiedName~EndpointManagerTests
```

`npm test` chains four suites (`test:companion`, `test:installer`, `test:native`, `test:firmware`); the last three need .NET SDK / cmake+ninja. Run `npm run test:companion` alone for a fast loop.

Companion runtime diagnostics are env vars, not build flags: `DS5_BRIDGE_DIAGNOSTICS=off|audio|traces|helper|all`, plus individual `DS5_BRIDGE_*_DIAGNOSTICS=1` overrides (`companion/src/main/debug-config.ts`).

`npm run visual:smoke` / `layout:check` drive Playwright over the built app — only run when explicitly asked.

## Architecture

### Firmware data flow

Core 0 runs one cooperative superloop in `main.cpp` under a 1 s hardware watchdog, stepping named phases (`WatchdogMainLoopPhase`) so a hang can be attributed after reboot via `watchdog_telemetry`: `cyw43_arch_poll` → `tud_task` → firmware log flush → `interrupt_loop` (bridge input/output) → USB power → `audio_loop` → BOOTSEL button → lightbar.

Core 1 is started by `audio_init()` **before** `cyw43_arch_init()`, because CYW43 init also initializes BTstack's flash-backed pairing store, and core 1 must already be servicing cooperative XIP pause requests before flash can be erased safely. Core 1 then runs the Opus/haptic audio pipeline out of RAM (`__not_in_flash_func`) with a canary-guarded stack.

Controller → host: `bt.cpp` (inquiry, pairing, L2CAP HID) → `dualsense_input_decoder` → `BridgeControllerState` → the active **persona** encodes a host report. Host → controller: host output report → persona decode to a DS5 payload → `controller_output_policy` (gain scaling, sanitizing, lightbar/mic/speaker overrides) → `controller_packet_compositor` → `output_scheduler` picks between the audio stream and urgent/coalesced state on the shared BT interrupt lane → `bt.cpp`.

`src/persona/` is the host-facing identity layer (`HostPersonaMode`: DualSense, DualSense Edge, DS4, Xbox 360/XUSB). Switching persona changes the USB configuration descriptor and re-enumerates; `host_input_prepare_persona_switch()` quiets input across the swap. XUSB is not native HID and has its own USB driver path (`xusb360_usb.cpp`, `usb_app_drivers.cpp`).

### Dual-controller support: DualSense *or* generic HID pad

One image handles both. Whichever controller connects first decides the session — a DualSense gets the full native flow (audio, haptics, adaptive triggers, lightbar); anything else is treated as a generic HID gamepad and presented as an Xbox 360 pad.

- **Classification** happens in `bt.cpp` from the DualSense `0x20` vendor feature probe. A `0xA3 0x20` answer means Sony (byte 23 `0x44` = Edge); a `0x02` HIDP NAK means *not* Sony; a `CONTROLLER_TYPE_DETECT_TIMEOUT_US` expiry means generic. Note the `0x02` reading is the reverse of what it once was: the probe used to be the Edge-only report `0x70`, which a base DualSense legitimately NAKs, so back then a NAK *did* mean DualSense.
- **`publish_controller_as()` is the only place a controller becomes visible to USB**, and the ordering inside it is the point: persona is latched **before** `usb_handle_controller_transport_ready()`. The bridge does not enumerate at boot, and descriptors are pulled from `host_persona_active()` at GET_DESCRIPTOR time, so a persona chosen there costs **no re-enumeration**. Publishing first and correcting after would enumerate the wrong identity.
- `bridge_mode_is_stellaris()` (`src/bridge_mode.h`) is the runtime gate. It is an `inline` accessor over an `inline` variable **kept in the header**, because three call sites (`on_bt_data`, `interrupt_loop`, `bt_write_audio_stream`) are SRAM-relocated and a *call* into flash-resident state from those would fault while core 1 has XIP paused. The flag must stay **non-`const`** — `const`/`constexpr` data lands in `.rodata`, which is flash. `src/bridge_latency.h` uses the same construction.
- Input: `src/hid_report_descriptor.cpp` parses a pad's HID report descriptor (fetched over SDP); `src/generic_hid_input_decoder.cpp` maps it onto the 21 fields XUSB reads. A table of layouts verified against hardware is matched on **report ID + report length** — the same pad uses different report shapes *and different button numbering* per pairing mode, so button maps travel with the layout.
- In generic mode, output to the pad is cut at the four `enqueue_*` funnels in `bt.cpp`, plus `bt_power_off_controller` (direct `l2cap_send`) and `audio_loop`. Rumble is the one exception: `bt_stellaris_set_rumble()` sends a raw HID output report via `bt_send_raw_hid_output()`, bypassing the DualSense sequence-nibble and CRC framing, with amplitude encoded by `src/switch_rumble.h`.
- `src/stellaris_diagnostics.cpp` types diagnostics through the companion keyboard HID interface (BOOTSEL double-press cycles: guided capture → rumble probe → stop). Inert while a DualSense is connected, which has the companion app instead.

`companion.cpp` implements the vendor HID companion interface (report IDs in `companion.h`) — status/input/audio-status telemetry out, commands + ACKs in, and it owns runtime settings dispatch. `host_bridge.cpp` is the separate vendor interface used by the WinUSB path — a bulk OUT endpoint plus vendor control requests (`0x31`/`0x32`) carrying both companion commands and streamed host audio (report `0x07`).

### Companion app processes

- `src/main/main.ts` — Electron main: window/tray/startup, and one `ipcMain.handle` per renderer channel.
- `src/main/bridge-service.ts` — the core (~160 KB): device discovery, transport lifecycle, command sequencing/ACKs, polling timers, chord→virtual-key mapping, profiles. Most behavior changes land here.
- `src/main/winusb-companion-transport.ts` — the actual bridge link. It spawns the .NET `AudioHelper` as a child process speaking a line-oriented JSON protocol; `node-hid` is used only for *enumeration*, in a forked `hid-discovery-worker` (a full HID enumeration blocks, so it is kept off the main process).
- `src/main/settings-store.ts` — persisted `CompanionSettings`, presets, profiles.
- `src/shared/protocol.ts` — the single TypeScript mirror of the firmware wire format: magic `DS5B`, `PROTOCOL_MAJOR`/`PROTOCOL_MINOR`, report IDs, command IDs, encoders/parsers. Must stay in lockstep with `src/companion.cpp` and `src/companion.h`.
- `src/renderer/App.tsx` — the whole UI in one very large file (~460 KB), styled by an equally large `styles.css`. Grep by feature/section name rather than trying to read either end to end.
- `companion/native/AudioHelper/` — audio session discovery, loopback capture for audio-reactive haptics, endpoint defaults, speaker test, mic keepalive, media metadata, icon extraction, and the WinUSB bridge/PCM transports.

## Invariants that will bite you

**Many tests assert against raw source text, not runtime behavior.** `tests/firmware/usb_descriptor_migration_test.cpp` (~2200 lines) reads `src/*.cpp`, `CMakeLists.txt`, `companion/src/main/bridge-service.ts`, and workflow files, then asserts dozens of cross-cutting policies (Bluetooth pairing/reconnect, watchdog + BOOTSEL flash safety, persona identity isolation, chord starter deferral, battery bucket handling, …). `diagnostics_config_test.cpp`, `ipc-contract.test.ts`, `app-behavior.test.ts`, and `styles-layout.test.ts` do the same for their areas. Renaming a constant, reordering statements, or reformatting can fail a test that has nothing to do with your change — read the failing assertion, then decide whether to restore the pattern or update the guard.

- **USB descriptors**: any change to `src/usb_descriptors.c` fails the migration guard. If intentional, bump `bcdDevice` so Windows re-enumerates cleanly, then update `kExpectedUsbDeviceRevision` and `kExpectedCompanionDescriptorHash`. VID/PID, string descriptors, interface order/count, and audio topology are Windows PnP identity — stale test identities need `tools/windows/clean-ds5bridge-devices.ps1`.
- **Firmware version has one canonical source**: `firmware-version.txt`. `CMakeLists.txt`, `src/companion.cpp`, `BUNDLED_FIRMWARE_VERSION` in `bridge-service.ts`, `tools/create-release-candidate.ps1`, and `.github/workflows/release.yml` must all agree; the guard enforces it.
- **Protocol versioning**: `kProtocolMajor`/`kProtocolMinor`/`kProtocolMinSupportedMinor` in `companion.cpp` and `PROTOCOL_MAJOR`/`PROTOCOL_MINOR` in `protocol.ts`. Firmware accepts the same major within `[minSupportedMinor, minor]`, and reads newer fields conditionally (`protocol_minor >= N`). Adding a command means bumping the minor on both sides and gating the new payload bytes — never repurpose existing offsets.
- **`verify_core1_sram.cmake` only checks symbol *placement*, never what those symbols *call*.** After touching anything inside an SRAM-relocated function, disassemble it and confirm every branch target is `0x2xxxxxxx` with no `_veneer` — a flash call from there faults while core 1 has XIP paused, and the script will not catch it:
  `arm-none-eabi-objdump -d build/pico2w/ds5-bridge.elf | awk '/<_Z10on_bt_data/,/^$/' | grep -E '\sbl\s|\sb\.w\s'`
- **Installer identity**: `build.appId` (`io.github.sundaymoments.ds5bridge`) and `build.nsis.guid` must never change, or users get side-by-side installs instead of an upgrade. `test:installer` guards this.
- **UI layout**: `companion/UI_STYLE_GUIDE.md` is a contract, not a suggestion — shared `:root` tokens, the `feature-heading` + `feature-card-grid` paired-card structure on every tab, `CustomSelect` instead of native `<select>`, `lucide-react` icons. Document deviations there first.
- **Supply chain**: `companion/.npmrc` blocks git deps and packages younger than three days. Keep `npm ci` from the lockfile.

## Conventions

Commits use short prefixes: `feat:`, `fix:`, `docs:`, `chore:`, `refactor:`, `test:`, `build:`, `ci:` (see `CONTRIBUTING.md`). Development happens on `port-dev`; `main` is the release branch, and beta releases always tag the `port-dev` head.

Further reading: `docs/development.md` (builds, packaging, releases), `docs/diagnostics.md` (UART wiring, presets, log collector), `docs/windows-device-cleanup.md`, `docs/persona-emulation-profile-research.md`.
