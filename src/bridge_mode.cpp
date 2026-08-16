//
// Bridge mode: which upstream controller family the bridge connects to.
//

#include "bridge_mode.h"

#include "host_input.h"
#include "usb.h"
#include "utils.h"

#include "pico/cyw43_arch.h"

namespace {

// The bridge boots in DualSense mode and is re-latched when a controller
// connects and turns out to be something else. Nothing enumerates on USB until
// that classification happens, so the boot value is never what the host sees.
constexpr BridgeMode kDefaultBridgeMode = BridgeModeDualSense;

// One source of truth. The flag in the header is what the SRAM hot paths read;
// this keeps the enum-typed view in step with it.
void store_bridge_mode(BridgeMode mode) {
    g_bridge_mode_stellaris = mode == BridgeModeStellaris;
}

// Acknowledgement blink. One pulse entering DualSense mode, two entering
// Stellaris mode, so the switch is legible with no controller connected and no
// companion app running. The onboard LED is the only indicator available while
// disconnected; the DualSense lightbar and player LEDs are not.
constexpr uint32_t kIndicatorPulseMs = 150;
constexpr uint8_t kIndicatorPulsesDualSense = 1;
constexpr uint8_t kIndicatorPulsesStellaris = 2;

uint8_t indicator_edges_remaining = 0;
bool indicator_led_on = false;
uint32_t indicator_next_edge_ms = 0;

// Wrap-safe comparison for short-lived 32-bit millisecond timestamps, matching
// the time_reached_u32 idiom in main.cpp.
bool indicator_time_reached(uint32_t now, uint32_t target) {
    return static_cast<int32_t>(now - target) >= 0;
}

void indicator_write(bool on) {
    indicator_led_on = on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

void indicator_start(BridgeMode mode) {
    // mute[0] is the companion "bridge LED enabled" flag; honour it exactly as
    // the inquiry blink in bt.cpp does.
    if (mute[0]) {
        indicator_edges_remaining = 0;
        return;
    }
    const uint8_t pulses = mode == BridgeModeStellaris
        ? kIndicatorPulsesStellaris
        : kIndicatorPulsesDualSense;
    // Two edges per pulse: on then off.
    indicator_edges_remaining = static_cast<uint8_t>(pulses * 2);
    indicator_next_edge_ms = 0;
}

} // namespace

BridgeMode bridge_mode_active() {
    return g_bridge_mode_stellaris ? BridgeModeStellaris : BridgeModeDualSense;
}

void bridge_mode_latch(BridgeMode mode) {
    if (mode == bridge_mode_active()) {
        return;
    }
    DS5_LOG(
        "[MODE] Controller classified; bridge mode %u -> %u\n",
        static_cast<unsigned int>(bridge_mode_active()),
        static_cast<unsigned int>(mode)
    );
    store_bridge_mode(mode);
}

HostPersonaMode bridge_mode_persona(BridgeMode mode) {
    return mode == BridgeModeStellaris
        ? HostPersonaModeXusb360
        : HostPersonaModeDualSense;
}

void bridge_mode_reset_to_default() {
    store_bridge_mode(kDefaultBridgeMode);
    indicator_edges_remaining = 0;
}

BridgeMode bridge_mode_set_active(BridgeMode mode) {
    if (mode == bridge_mode_active()) {
        return bridge_mode_active();
    }

    const HostPersonaMode target_persona = bridge_mode_persona(mode);
    if (!host_persona_is_supported(target_persona)) {
        DS5_LOG(
            "[MODE] Refusing switch to mode %u; persona %u unsupported\n",
            static_cast<unsigned int>(mode),
            static_cast<unsigned int>(target_persona)
        );
        return bridge_mode_active();
    }

    DS5_LOG(
        "[MODE] Switching bridge mode %u -> %u (persona %u)\n",
        static_cast<unsigned int>(bridge_mode_active()),
        static_cast<unsigned int>(mode),
        static_cast<unsigned int>(target_persona)
    );

    // The Bluetooth controller is deliberately NOT disconnected here.
    //
    // Disconnecting looks harmless but destroys the USB device. The HCI
    // disconnect completes asynchronously and lands in
    // usb_handle_controller_transport_disconnect(), which clears
    // usb_reconnect_requested -- cancelling the re-enumeration queued below --
    // and clears usb_controller_transport_ready. usb_pm_poll then sees a
    // not-ready transport, and because wake retention is disabled for XUSB it
    // hard-detaches with tud_disconnect(). The bridge disappears from the host
    // entirely.
    //
    // This path is the post-enumeration swap, and re-enumeration is what makes
    // it work. The connect-time path does not come through here: it calls
    // bridge_mode_latch() before the bus is ever attached, so there is nothing
    // to re-enumerate.
    store_bridge_mode(mode);

    const bool persona_changed = host_persona_active() != target_persona;
    if (persona_changed) {
        host_input_prepare_persona_switch();
        if (!host_persona_set_active(target_persona)) {
            DS5_LOG("[MODE] Persona activation failed; mode is now %u without re-enumeration\n",
                    static_cast<unsigned int>(mode));
            indicator_start(bridge_mode_active());
            return bridge_mode_active();
        }
        usb_request_reconnect();
    }

    indicator_start(bridge_mode_active());
    return bridge_mode_active();
}

BridgeMode bridge_mode_toggle() {
    // Mode follows whichever controller connects, so there is nothing for a
    // manual toggle to decide -- forcing the wrong one would only break decode
    // until the next reconnect. The BOOTSEL double-press that lands here drives
    // the diagnostic dump instead.
    return bridge_mode_active();
}

void bridge_mode_indicator_poll(uint32_t now_ms) {
    if (indicator_edges_remaining == 0) {
        return;
    }
    if (!indicator_time_reached(now_ms, indicator_next_edge_ms)) {
        return;
    }

    indicator_edges_remaining--;
    indicator_write(!indicator_led_on);
    indicator_next_edge_ms = now_ms + kIndicatorPulseMs;

    if (indicator_edges_remaining == 0) {
        // Leave the LED off and let bt.cpp reassert inquiry blink or the
        // solid connected state on its next transition.
        indicator_write(false);
    }
}
