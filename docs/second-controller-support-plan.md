# Second Simultaneous Controller Support Plan

Feasibility analysis and implementation plan for connecting a second Bluetooth
controller to the bridge at the same time as the DualSense. The concrete case
driving this document is a **Cosmic Byte Stellaris** used as a secondary pad, so
that its bundled 2.4 GHz dongle no longer occupies a USB port.

Status: **research complete, not started.** Step 0 gates all implementation work.

## Context

The firmware is architecturally single-controller and DualSense-specific in four
independent layers, and each must change:

1. **Radio** — BTstack is linked Classic-only (`CMakeLists.txt:488`) with
   `MAX_NR_HCI_CONNECTIONS 1` (`src/btstack_config.h:19`), and *five* independent
   firmware gates reject a second link.
2. **Decode** — hardcoded DualSense byte offsets
   (`src/dualsense_input_decoder.cpp:66-136`). The firmware has **no SDP client and
   never parses a HID report descriptor** — it never asks a controller what it is.
3. **Host presentation** — one gamepad interface, one global `HostPersonaMode`.
4. **Companion protocol/app** — status and identity payloads describe exactly one
   controller, and both are already byte-full.

**Blocking unknown:** whether the Stellaris's Bluetooth mode is Bluetooth Classic
HID or BLE HID (HOGP). Evidence points to BLE — the manual lists Bluetooth mode at
**125 Hz** polling versus 1000 Hz wired/2.4 GHz, and ~8 ms is the BLE
minimum-connection-interval signature; Classic HID sustains 250–1000 Hz (the
DualSense does). **If it is BLE, the entire BLE stack must be added** to a build
that currently has ~8 KiB of runtime heap headroom.

**Cheapest alternative, recorded for completeness.** The Stellaris is a generic HID
gamepad and needs none of the DualSense-specific bridging this firmware exists to
provide. Pairing it directly to a host PC's own Bluetooth radio frees the USB port
with zero firmware work. This plan exists because that path was explicitly declined.

---

## Step 0 — Identify the controller's Bluetooth protocol (do this first)

Three checks. Any one is suggestive; agreement across two is conclusive.

### Check A — Windows pairing dialog (2 min)

1. Put the controller in Bluetooth mode (`A + HOME` for D-Input, per its manual).
2. **Settings → Bluetooth & devices → Add device**, observe how it pairs.

| Observation | Verdict |
| --- | --- |
| Pairs instantly, no PIN/passkey | Likely BLE |
| Pairing shows a PIN/passkey exchange | Likely Classic |
| Disappears from the list quickly when idle | Strongly BLE (advertising interval) |

### Check B — Phone BLE scanner (definitive, 3 min)

1. Install **nRF Connect for Mobile** (Nordic Semiconductor, free).
2. Put the controller in pairing mode, then **Scan**.
3. **If the controller appears, it is BLE.** nRF Connect scans BLE advertising only —
   a Bluetooth Classic device is invisible to it.
4. If it appears, tap it and check for service **Human Interface Device (0x1812)**,
   which confirms HOGP/BLE HID.

### Check C — Windows hardware ID (confirms, and captures IDs needed later)

**Device Manager → View → Devices by connection →** the controller **→ Properties →
Details → Hardware Ids**:

- `BTHLE\Dev_...` → **BLE**
- `BTHENUM\Dev_...` → **Bluetooth Classic**

Also record the **Bluetooth device address** and reported **VID/PID**.

### Record before proceeding

- Branch: **BLE HID** or **Bluetooth Classic HID**
- The `BTHLE\` / `BTHENUM\` string
- Whether service `0x1812` was listed
- Which mode was active — test **D-Input** first, since X-Input over Bluetooth is
  often a vendor-specific protocol rather than plain HID

---

## Feasibility verdict by branch

### Branch CLASSIC — hard but tractable

Reuses the existing L2CAP HID plumbing (PSM 0x11/0x13 services are already registered
globally at `src/bt.cpp:2541-2542` and are *not* per-connection). Work is Phases A–D.

### Branch BLE — substantially harder; re-scope before committing

Everything in Branch CLASSIC **plus** a second protocol stack:

- link `pico_btstack_ble`, define `ENABLE_BLE` + `ENABLE_LE_CENTRAL`
  (`src/btstack_config.h`)
- add GATT client, HIDS/HOGP client, LE Security Manager + bonding,
  `MAX_NR_GATT_CLIENTS`, `MAX_NR_SM_LOOKUP_ENTRIES`, LE ACL buffers
- run **dual-mode** Classic + LE concurrently on the CYW43439, whose scheduler must
  interleave LE connection events against the DualSense's latency-sensitive audio

Static cost lands squarely on a budget with **8 KiB** of guaranteed post-startup heap
(`CMakeLists.txt:500-504`). Branch BLE needs its own feasibility spike before any
delivery commitment; it is not a straight extension of Branch CLASSIC.

---

## Phase A — Multi-connection Bluetooth foundation

The bulk of the risk. Must land first, proven with **two DualSenses**, before any
Stellaris-specific work starts. That isolates "can the radio hold two links" from
"can we decode an unknown controller" — two hard problems that otherwise fail in
ways that are difficult to tell apart.

### A1. Raise BTstack limits — `src/btstack_config.h`

| Macro | Now | Target | Cost |
| --- | --- | --- | --- |
| `MAX_NR_HCI_CONNECTIONS` | `1` (`:19`) | `2` | ≈1.2 KB static (each `hci_connection_t` holds a 1037-byte ACL recombination buffer) |
| `MAX_NR_L2CAP_CHANNELS` | `2` (`:20`) | `4` | two more `l2cap_channel_t` |
| `MAX_NR_L2CAP_SERVICES` | `3` (`:21`) | **unchanged** | services are per-PSM, not per-connection |
| `NVM_NUM_LINK_KEYS` | `4` (`:34`) | **unchanged** | already stores 4 bonds |

`HAVE_MALLOC` is not defined, so all BTstack pools are fixed static arrays sized by
these macros — growth is entirely static and is caught by the build assertion in A4.

### A2. Convert `bt.cpp` session state to a per-link struct

`src/bt.cpp:384-490` is a flat block of file-scope singletons with no connection
struct. Introduce `struct BtControllerSession { ... }` and an array of 2, keyed by
`hci_con_handle_t`, moving in at minimum:

`current_device_addr`, `acl_handle`, `hid_control_cid`, `hid_interrupt_cid`, both
pending CIDs, `hid_control_ready`/`hid_interrupt_ready`, `controller_type`,
`connection_phase` + `connection_generation`, the encryption/authentication/disconnect
retry blocks, the entire RSSI block (`:441-449`), `feature_data` + prefetch queue,
**the whole TX scheduler** (`urgent_queue`, `audio_queue`, `interrupt_send_packet`,
`control_queue`, pacing counters `:455-469`), `state_pending_report[78]`,
`state_report_seq`, `inactive_time`, and `classic_rumble_state`.

Leave genuinely global: inquiry/pairing-window state, the CYW43 LED, blacklist, user
settings, `queue_lock`.

Also per-connection but in another translation unit:
`src/controller_output_state.cpp:17-37` (shadow 0x31 payload + adaptive-trigger cache)
must be instanced per controller.

### A3. Open the five gates that reject a second connection

All in `src/bt.cpp`:

1. `classic_acl_connection_allowed()` `:1492-1545` — the phase gate `:1493-1500` and
   the "another Classic transaction owns the radio" check `:1510-1520` must become
   *per-session-slot* checks ("is there a **free** slot") rather than global.
2. `classic_connection_filter()` `:1547-1549` — same.
3. `HCI_EVENT_CONNECTION_REQUEST` `:3498-3524` — re-checks both of the above.
4. **Radio goes non-connectable while connected** — `restore_passive_reconnect_scan()`
   `:1403-1415` gates on `acl_handle == HCI_CON_HANDLE_INVALID`, and
   `gap_connectable_control(0)` fires at `:3251` and `:3926`. This must become
   "connectable while any slot is free". *This is the single most important
   behavioural change* — today a second controller cannot even page the dongle.
5. `L2CAP_EVENT_INCOMING_CONNECTION` `:4194-4232` — declines unless the handle matches
   the one `acl_handle`; must resolve to the owning session.

Also make the pairing transaction record (TLV `'PTX2'`, `:153-156`, `:342-349`) either
multi-slot or explicitly serialized, since only one pairing may be in flight
device-wide.

### A4. Respect the SRAM/heap build assertion

`cmake/verify_core1_sram.cmake` runs POST_BUILD and hard-fails if post-startup heap
drops below **87,312 bytes** (`CMakeLists.txt:500-504`). Two consequences:

- The heap-allocated TX scheduler (`std::deque` `urgent_queue`, `std::vector`
  `control_queue`, `std::unordered_map` `feature_data`, and each packet's `data`) is
  duplicated per controller and eats directly into the **8 KiB** slack. Strongly prefer
  converting the second session's queues to **fixed static storage**, following the
  existing `ExactAudioQueue` pattern (`src/audio_exact_queue.h:12-24`), which was
  written precisely to avoid heap use.
- The same script pins ~130 **mangled** hot-path symbols into SRAM, including
  `_Z10on_bt_data12CHANNEL_TYPEPht`, `l2cap_send`, and `hci_run`. Adding a device
  parameter to the `bt_data_callback_t` signature (`src/bt.h:41`) **changes that
  mangled name and fails the build** until `verify_core1_sram.cmake` is updated in the
  same commit. Budget for this — it presents as an unrelated build break.

### A5. Bandwidth and audio contention (the real risk)

The DualSense link already carries input **plus** Opus speaker audio **plus** mic, and
the TX scheduler (`src/output_scheduler.h`) exists specifically to arbitrate that one
link. A second ACL link shares piconet airtime. Expect to revisit the scheduler and to
measure rather than assume. `MAX_NR_HCI_ACL_PACKETS` (`:17`) is dead config — it is
referenced nowhere in btstack, so tuning it will not help.

**Exit criterion for Phase A:** two DualSenses connected simultaneously, both
delivering input, with audio and haptics on controller 1 measurably no worse than
today (`DS5_DIAGNOSTICS_PRESET=traces` plus the `[FB]` CSV trace).

---

## Phase B — Support a non-DualSense controller

The firmware currently *cannot describe* an unknown controller.
`src/dualsense_input_decoder.cpp:66-136` is straight-line offset decoding of a 63-byte
DualSense payload; `src/main.cpp:490-506` asserts BT report ID `0x31` at `data[1]` and
slices the payload from offset 3. A repo-wide search for `sdp_client`,
`hid_descriptor`, `report_descriptor`, and `hid_host` returns **zero hits** —
`sdp_init()` (`bt.cpp:2540`) starts the SDP *server* only, with no records registered.

Two options:

- **B-hardcode (recommended first):** capture the controller's actual report layout
  once (Wireshark/btmon on the PC, or a debug build dumping raw interrupt payloads over
  UART per `docs/diagnostics.md`), then write a decoder mirroring the DualSense one,
  plus a `ControllerType` value and a detection rule. Cheap; brittle against controller
  firmware revisions.
- **B-generic:** add an SDP client query for the HID descriptor and a real
  report-descriptor parser producing a usage→field map. The "right" answer; roughly
  doubles Phase B. Note this moves toward generic-controller support, which the project
  has so far deliberately left to Kitsune Input — `docs/devices-tab-port-plan.md`
  explicitly excludes Switch Pro support.

Detection today is a vendor feature-report sniff (`bt.cpp:4070-4102`: `0xA3`/`0x20`,
byte 23 == `0x44` ⇒ Edge). A non-Sony device will never answer it, so the
type-detection path needs an explicit "unknown/generic" branch instead of falling
through to "assume DualSense" (`:4087-4092`).

---

## Phase C — Expose the second gamepad to Windows

**USB resources are not the blocker.** Companion builds expose 6 interfaces
(`bNumInterfaces = 0x06`, `src/usb_descriptors.c:183`) consuming endpoints `0x01` OUT,
`0x82` IN, `0x03` OUT, `0x84` IN, `0x86` IN, `0x07` OUT — about **896 bytes of the
RP2350's 3712-byte usable USB DPRAM**. A second gamepad interrupt IN + interrupt OUT
costs 128 bytes and 2 endpoint numbers; endpoint **5** is free in both directions.

Required mechanical changes:

- `CFG_TUD_HID` **2 → 3** (`src/tusb_config.h:101-105`)
- `bNumInterfaces` `0x06 → 0x07` (`usb_descriptors.c:183`) and
  `CONFIG_TOTAL_LEN_COMPANION` (`:42`) — otherwise the `TU_VERIFY_STATIC` at `:604`
  fails at **compile** time, before tests run
- `src/companion.h:8` `#define KEYBOARD_HID_INSTANCE 1` is **dead code and a trap** —
  nothing references it; all live code uses `host_persona_keyboard_hid_instance()`. A
  second gamepad shifts HID instance indices. Delete it rather than leave a stale
  constant that looks authoritative.

Three genuinely awkward spots:

1. **XUSB reuses the gamepad's endpoints.** `desc_xusb360_gamepad_interface[]`
   (`usb_descriptors.c:616-649`) is spliced *in place of* interface 3 and reuses
   `XUSB360_EP_IN 0x84` / `XUSB360_EP_OUT 0x03` (`:73-74`). It is not an additional
   interface.
2. **The runtime descriptor patcher scans for one interface.**
   `apply_gamepad_hid_runtime_configuration()` (`:661-688`) locates the gamepad by
   scanning for `GAMEPAD_INTERFACE_NUMBER` and the literal endpoint addresses
   `0x84`/`0x03` (`:682`). It must learn to patch two gamepads.
3. **Input send is hardwired to HID instance 0.**
   `host_input_send_report_for_persona()` (`src/main.cpp:350-364`) calls
   `tud_hid_report(...)` with no instance argument, and `host_input_ready_for_persona()`
   (`:346-348`) calls `tud_hid_ready()` likewise. On the output side,
   `tud_hid_set_report_cb()` (`:598-686`) casts `itf` to `(void)` at `:600` and only
   filters the keyboard — **anything that is not the keyboard is treated as *the*
   gamepad.** Both directions need real instance discrimination.

**Persona scoping is the design decision.** `HostPersonaMode` is a single global
(`src/persona/host_persona.cpp:9`) driving VID/PID, configuration descriptor, strings,
and the HID report descriptor for the whole USB device; changing it triggers
`usb_request_reconnect()` (`src/companion.cpp:2467-2487`). Give controller 2 a **fixed**
persona — strongly recommended — rather than a user-selectable one, or the
re-enumeration matrix becomes unmanageable.

**Descriptor guard — expect a cluster of deliberate updates.** In
`tests/firmware/usb_descriptor_migration_test.cpp`:

- `kExpectedUsbDeviceRevision` (`:14`) — bump `bcdDevice` (`usb_descriptors.c:142`) so
  Windows re-enumerates instead of reusing a cached descriptor, then update this.
- `kExpectedCompanionDescriptorHash` (`:15`) — recompute; it FNV-1a-64s the device
  descriptor, configuration array, MS-OS-2.0 descriptor, and string table.
- If `GAMEPAD_INTERFACE_NUMBER` moves, recompute `XUSB360_INTERFACE_DESC_FNV1A32` for
  **both** companion and non-companion variants (`usb_descriptors.c:68-72`) — the XUSB
  interface bytes embed the interface number, so a stale constant makes
  `host_persona_descriptors_verified(HostPersonaModeXusb360)` return false at runtime.
- **Watch the `extract_between` markers.** The guard slices source by exact literal
  strings — e.g. `host_persona_encode_input` must be followed by
  `"\n}\n\nbool host_persona_decode_output_to_ds5_payload"`. Refactoring those functions
  to take a controller index **throws a `std::runtime_error` rather than failing an
  assertion**, which reads as a broken test rather than a policy violation. Same for
  `tud_hid_get_report_cb` in `main.cpp`, sliced on
  `"\n}\n\n// Invoked when received SET_REPORT"`.

---

## Phase D — Companion protocol and app

**The payloads are already full — this is the key constraint.**
`COMPANION_PAYLOAD_SIZE` is 63 (`src/companion.h:20`), and both reports consume every
byte:

- **STATUS** (`0x01`) runs out to byte 63 (`quietModeEnabled` at
  `companion/src/shared/protocol.ts:853`).
- **DEVICE_IDENTITY** (`0x0d`) ends exactly at byte 63 — flags `[7]`, link-key type
  `[8]`, BT address ASCII 18 B `[9]`, name 24 B `[27]`, VID `[51]`, PID `[53]`, board
  id 8 B `[55]` (`src/companion.cpp:1768-1796`).

A second controller therefore **cannot** be appended to the existing reports. Add new
report IDs (e.g. `STATUS_2` / `DEVICE_IDENTITY_2`) or introduce a controller-index
selector byte and have the firmware answer per index. Decide this before writing any of
Phase D — it is the shape of the whole protocol change.

Also note: `BridgeStatusPayload` lists an `rssi` field but **no RSSI byte is actually
parsed from STATUS** (`protocol.ts:820+`). Confirm where signal strength really comes
from before duplicating it per controller.

Remaining work:

- Protocol version bump: `kProtocolMajor`/`kProtocolMinor` (`src/companion.cpp:33-35`)
  and the TypeScript mirror (`protocol.ts:6-7`, currently minor 22 — and pinned by the
  guard at `usb_descriptor_migration_test.cpp:1392`), gating new fields behind
  `protocol_minor >=` checks as existing code does.
- `SET_BUTTON_REMAP` (`0x1E`), `SET_CHORD_BINDINGS` (`0x23`), and
  `SET_RADIAL_DEADZONES` (`0x37`) payload builders emit flat arrays with **no target
  selector** (`protocol.ts:436-450`); the command frame has no target field either
  (`:1172`). All need a controller selector.
- Firmware-side these are single globals: `button_remap[]` (`companion.cpp:392`),
  `dynamic_chord_bindings[]` (`:278-279`), and the two deadzone percents (`:346-347`).
- `BridgeService` (`companion/src/main/bridge-service.ts`) tracks one snapshot; it must
  become an array, and `reapplySettingsUntilSettled()` (`:3909`) must reapply per
  controller.
- Renderer + `settings-store.ts`: profiles, remaps, chords, and deadzones all become
  per-controller. This is the largest single chunk of app work.

---

## Hard constraints to keep in view

| Constraint | Where | Consequence |
| --- | --- | --- |
| 8 KiB guaranteed post-startup heap | `CMakeLists.txt:500-504` | Prefer static storage for all new per-session state |
| ~130 mangled symbols pinned to SRAM | `cmake/verify_core1_sram.cmake` | Renaming or re-signaturing hot-path functions fails the build |
| Audio owns ~57 KB static + the 79 KB startup heap | `src/audio.cpp:208-234` | No meaningful SRAM headroom without cutting audio buffers |
| Core-1 stack overflow panics at runtime | `src/audio.cpp:793-797` | Real guard, not advisory |
| Source-scanning test guards | `tests/firmware/usb_descriptor_migration_test.cpp` | Many Bluetooth/persona policies are asserted against raw source text |
| Guard slices source on exact literals | same, `extract_between` | Refactoring `host_persona_encode_input` / `tud_hid_get_report_cb` **throws** instead of asserting — presents as a broken test |
| Companion payloads are 63/63 bytes full | `src/companion.h:20`, `companion.cpp:1768-1796` | A second controller needs new report IDs, not extended payloads |
| Only one controller can own speaker/mic | `src/audio.cpp` | Audio must be explicitly bound to controller 1; decide and document |

---

## Verification

1. **Phase A gate (two DualSenses):**
   `cmake -S . -B build/companion -G Ninja -DENABLE_COMPANION=ON -DDS5_DIAGNOSTICS_PRESET=traces`,
   flash, connect both, confirm both stream input and that controller 1's audio and
   haptics are unchanged. Capture `[FB]` traces before and after and compare drop
   counters.
2. **Firmware logic tests:**
   `cmake -S tests/firmware -B build-firmware-tests -G Ninja && cmake --build build-firmware-tests && ctest --test-dir build-firmware-tests --output-on-failure`
3. **Descriptor guard:** expect `usb_descriptor_migration_test` to fail after Phase C.
   Update `bcdDevice` and `kExpectedCompanionDescriptorHash` deliberately — never by
   copying the printed value without understanding the diff.
4. **Companion:** `cd companion && npm run typecheck && npm run test:companion`.
5. **Windows re-enumeration:** after any descriptor change, run
   `tools/windows/clean-ds5bridge-devices.ps1` to clear stale PnP entries before
   retesting.
6. **Soak:** both controllers connected ≥30 min with audio active, confirming no
   watchdog reboots (`watchdog_telemetry` phase reporting) and no core-1 stack panic.
