#ifndef DS5_BRIDGE_STELLARIS_DIAGNOSTICS_H
#define DS5_BRIDGE_STELLARIS_DIAGNOSTICS_H

#include <cstdint>

//
// Types what the bridge knows about the connected pad into whatever text field
// has focus.
//
// The Stellaris image has no other way out: the companion app shows a
// DualSense-shaped view that means nothing here, and UART needs an adapter
// soldered on. The keyboard HID interface is already enumerated in XUSB persona
// (host_persona_keyboard_hid_instance() returns 0 there), so it costs nothing
// to reuse it.
//
// Triggered by a BOOTSEL double-press, which in this image no longer has a
// bridge mode to switch to. Inert while a DualSense is connected -- that pad
// has the companion app as its diagnostic surface.
//

// Captures a snapshot and begins typing. Ignored while a dump is in progress.
void stellaris_diagnostics_request_dump();

// Emits at most one key event. Driven from button_check() so it shares the
// existing watchdog phase; never blocks.
void stellaris_diagnostics_poll(uint32_t now_ms);

// Abandons any run in progress. Called when a pad connects, and automatically
// when one drops: a capture that outlives its controller would keep typing into
// whatever window has focus, and would resume mid-run against a stale baseline
// once a pad came back.
void stellaris_diagnostics_reset();

#endif // DS5_BRIDGE_STELLARIS_DIAGNOSTICS_H
