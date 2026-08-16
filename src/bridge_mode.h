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
// Mode is intentionally not persisted. It is chosen when a controller connects,
// from what that controller turns out to be, and reset on disconnect.
enum BridgeMode : uint8_t {
    BridgeModeDualSense = 0,
    BridgeModeStellaris = 1,
};

//
// The gate every DualSense-only path is guarded on.
//
// Declared here as an inline variable plus an inline accessor for a specific
// reason: three of the gated call sites -- on_bt_data, interrupt_loop and
// bt_write_audio_stream -- are relocated into SRAM because they can run while
// core 1 has XIP paused for a flash write. A *call* from one of those into the
// flash-resident bridge_mode_active() would fault. Keeping the body in the
// header means the read inlines into the caller and no call is emitted at all.
// src/bridge_latency.h uses the same construction for the same reason.
//
// The flag must stay non-const: const and constexpr data is linked into
// .rodata, which lives in flash, and reading it during an XIP pause faults
// exactly like a call would. Plain mutable globals land in .data/.bss, which
// are SRAM, and need no placement attribute.
//
inline bool g_bridge_mode_stellaris = false;

inline bool bridge_mode_is_stellaris() {
    return g_bridge_mode_stellaris;
}

// Out-of-line accessor for the flash-resident callers. Never call this from an
// SRAM-relocated function; use bridge_mode_is_stellaris() there.
BridgeMode bridge_mode_active();

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

// Latches the mode from what a controller turned out to be, without touching
// USB. Used at classification time, before the bus is attached, so the decode
// and output gates can never disagree with the persona that is about to
// enumerate.
void bridge_mode_latch(BridgeMode mode);

// Services the onboard-LED acknowledgement blink. Safe to call at any cadence;
// driven from button_check() so it shares the existing 100 ms poll and does not
// need its own watchdog phase.
void bridge_mode_indicator_poll(uint32_t now_ms);

#endif // DS5_BRIDGE_BRIDGE_MODE_H
