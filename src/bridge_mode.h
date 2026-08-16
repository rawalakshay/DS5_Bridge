#ifndef DS5_BRIDGE_BRIDGE_MODE_H
#define DS5_BRIDGE_BRIDGE_MODE_H

#include <cstdint>

#include "persona/host_persona.h"

// Which upstream controller family the bridge talks to over Bluetooth.
//
// This is deliberately separate from HostPersonaMode: persona describes what
// the bridge presents to the USB host, while bridge mode describes what the
// bridge connects to. Mode selects the input decoder and gates the
// DualSense-only output, audio, and haptics paths.
//
// Mode is intentionally not persisted. restore_defaults() returns it to
// BridgeModeDualSense on every boot, matching the persona reset that already
// lives there.
enum BridgeMode : uint8_t {
    BridgeModeDualSense = 0,
    BridgeModeStellaris = 1,
};

BridgeMode bridge_mode_active();

// The gate every DualSense-only path is guarded on.
//
// Deliberately compile-time in both directions rather than a read of the
// runtime mode. Several of the gated call sites -- on_bt_data,
// bt_write_audio_stream, the output enqueue funnels -- are relocated into SRAM
// because they can run while core 1 has XIP paused for a flash write, and a
// call from one of those into flash-resident mode state would fault. Folding
// the branch at compile time removes the call entirely, and leaves the
// DualSense image's code generation untouched.
constexpr bool bridge_mode_is_stellaris() {
#if STELLARIS_ONLY
    return true;
#else
    return false;
#endif
}

// The host persona a mode presents to Windows.
HostPersonaMode bridge_mode_persona(BridgeMode mode);

// Returns the mode to its boot default without touching USB or Bluetooth.
// The companion restore-defaults path owns its own persona reset, so this must
// not re-enumerate on its own.
void bridge_mode_reset_to_default();

// Disconnects the current controller, swaps the host persona, and re-enumerates
// USB. No-op when already in the requested mode or when the target persona is
// unsupported. Returns the mode in effect afterwards.
BridgeMode bridge_mode_set_active(BridgeMode mode);
BridgeMode bridge_mode_toggle();

// Services the onboard-LED acknowledgement blink. Safe to call at any cadence;
// driven from button_check() so it shares the existing 100 ms poll and does not
// need its own watchdog phase.
void bridge_mode_indicator_poll(uint32_t now_ms);

#endif // DS5_BRIDGE_BRIDGE_MODE_H
