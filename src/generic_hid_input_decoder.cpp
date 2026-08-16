//
// Generic Bluetooth gamepad input decoding. See generic_hid_input_decoder.h.
//

#include "generic_hid_input_decoder.h"

#include <cstring>

namespace {

constexpr uint16_t kLastReportCapacity = 32;

// Button number (1-based, as the HID Button page numbers them) to the field the
// XUSB persona reads.
//
// The report descriptor says where each button lives but not what it is called
// -- it only ever declares "Button 1..N". This table is that missing half, and
// follows the ordering used by essentially every generic Bluetooth gamepad in
// D-input/Android mode. If a pad comes back with two buttons transposed, this
// table is the one place to fix it.
struct ButtonBinding {
    uint8_t number;
    bool BridgeControllerState::*field;
};

// Sparse numbering: leaves gaps at 3 and 6, the positions a six-face-button
// layout would have used for C and Z. Standard for Android/D-input pads.
constexpr ButtonBinding kAndroidButtonMap[] = {
    {1, &BridgeControllerState::cross},       // A
    {2, &BridgeControllerState::circle},      // B
    {4, &BridgeControllerState::square},      // X
    {5, &BridgeControllerState::triangle},    // Y
    {7, &BridgeControllerState::l1},
    {8, &BridgeControllerState::r1},
    {9, &BridgeControllerState::l2_pressed},
    {10, &BridgeControllerState::r2_pressed},
    {11, &BridgeControllerState::create},     // Back / Select
    {12, &BridgeControllerState::options},    // Start
    {13, &BridgeControllerState::home},       // Guide
    {14, &BridgeControllerState::l3},
    {15, &BridgeControllerState::r3},
};

// Dense numbering, 1..12 with no gaps: the classic DirectInput gamepad order.
// The same pad uses this in Windows mode and the sparse map above in Android
// mode, so the button map has to travel with the layout rather than being
// global.
constexpr ButtonBinding kDirectInputButtonMap[] = {
    {1, &BridgeControllerState::cross},       // A
    {2, &BridgeControllerState::circle},      // B
    {3, &BridgeControllerState::square},      // X
    {4, &BridgeControllerState::triangle},    // Y
    {5, &BridgeControllerState::l1},
    {6, &BridgeControllerState::r1},
    {7, &BridgeControllerState::l2_pressed},
    {8, &BridgeControllerState::r2_pressed},
    {9, &BridgeControllerState::create},      // Back / Select
    {10, &BridgeControllerState::options},    // Start
    {11, &BridgeControllerState::l3},
    {12, &BridgeControllerState::r3},
    {13, &BridgeControllerState::home},
};

constexpr uint8_t kAndroidButtonMapCount =
    static_cast<uint8_t>(sizeof(kAndroidButtonMap) / sizeof(kAndroidButtonMap[0]));
constexpr uint8_t kDirectInputButtonMapCount =
    static_cast<uint8_t>(sizeof(kDirectInputButtonMap) / sizeof(kDirectInputButtonMap[0]));

// Used only until the descriptor arrives, or if the SDP query fails outright.
//
//   byte 0   report ID
//   byte 1   left stick X        byte 2   left stick Y
//   byte 3   right stick X       byte 4   right stick Y
//   byte 5   hat, 0..7 clockwise from north, 0x0F when centred
//   byte 6   buttons 1-8         byte 7   buttons 9-16
//   byte 8   R2 analog           byte 9   L2 analog
//
// Verified byte for byte against a Cosmic Byte Stellaris in Android mode, and
// the most common shape for this class of pad generally. Note the analog
// triggers are in the reverse order to the buttons -- right before left -- and
// the pad reports each trigger both as a button and as an axis.
//
// The report ID is not pinned here: it is adopted from the first report
// received, because it is the one field that reliably differs between devices
// and between pairing modes on the same device.
constexpr HidGamepadLayout kFallbackLayout = {
    /* valid            */ true,
    /* uses_report_id   */ true,
    /* report_id        */ 0,
    /* report_bits      */ 80,
    /* x  */ {true, 0, 8, 0, 255},
    /* y  */ {true, 8, 8, 0, 255},
    /* z  */ {true, 16, 8, 0, 255},
    // rx and ry carry the triggers, not a stick: the decoder routes whichever
    // axis pair the right stick does not claim to the triggers.
    /* rx */ {true, 64, 8, 0, 255},
    /* ry */ {true, 56, 8, 0, 255},
    /* rz */ {true, 24, 8, 0, 255},
    /* hat */ {true, 32, 8, 0, 7},
    /* buttons_present    */ true,
    /* button_bit_offset  */ 40,
    /* button_count       */ 16,
};

// Layouts confirmed against real hardware, matched on report ID and length.
//
// The Stellaris does not use one format across its pairing modes -- it changes
// the report ID, the length, the field order, and even the axis width. Nothing
// short of a per-mode layout works, which is why the report descriptor would
// have been the better answer had the pad published one.
struct KnownLayout {
    char const *name;
    uint8_t report_id;
    uint16_t report_len; // Total bytes on the wire, report ID included.
    HidGamepadLayout layout;
    ButtonBinding const *buttons;
    uint8_t button_map_count;
};

constexpr KnownLayout kKnownLayouts[] = {
    // Stellaris, Android mode. 8-bit axes, analog triggers on bytes 8 and 9
    // (right before left), hat as a full byte, sparse button numbering.
    {"STELLARIS ANDROID", 0x07, 11, kFallbackLayout,
     kAndroidButtonMap, kAndroidButtonMapCount},

    // Stellaris in Windows *or* Nintendo mode -- both produce this identical
    // report, and both accept the Switch 0x10 rumble frame.
    //
    // This is the Nintendo Switch Pro "simple HID" report: buttons, hat, then
    // four 16-bit little-endian axes centred on 0x8000. Verified byte for byte
    // in both modes. No analog triggers -- the shoulder triggers report as
    // buttons only -- and dense DirectInput button numbering rather than the
    // sparse Android one.
    {"SWITCH HID (WINDOWS+NINTENDO)", 0x3F, 12,
     {
         /* valid            */ true,
         /* uses_report_id   */ true,
         /* report_id        */ 0x3F,
         /* report_bits      */ 88,
         /* x  */ {true, 24, 16, 0, 65535},
         /* y  */ {true, 40, 16, 0, 65535},
         /* z  */ {true, 56, 16, 0, 65535},
         /* rx */ {false, 0, 0, 0, 0},
         /* ry */ {false, 0, 0, 0, 0},
         /* rz */ {true, 72, 16, 0, 65535},
         /* hat */ {true, 16, 8, 0, 7},
         /* buttons_present    */ true,
         /* button_bit_offset  */ 0,
         /* button_count       */ 16,
     },
     kDirectInputButtonMap, kDirectInputButtonMapCount},
};

constexpr uint8_t kKnownLayoutCount =
    static_cast<uint8_t>(sizeof(kKnownLayouts) / sizeof(kKnownLayouts[0]));

HidGamepadLayout active_layout = kFallbackLayout;
HidLayoutSource layout_source = HidLayoutSource::Generic;
char const *layout_name = "GENERIC GUESS";
bool layout_locked = false;

// The descriptor and generic paths have no button naming of their own, so they
// use the Android numbering: it is what most Bluetooth pads follow.
ButtonBinding const *active_button_map = kAndroidButtonMap;
uint8_t active_button_map_count = kAndroidButtonMapCount;

uint8_t last_report[kLastReportCapacity];
uint16_t last_report_len = 0;
uint32_t last_button_mask = 0;
uint16_t last_hat_value = kHidHatCentred;
uint8_t seen_report_ids[kHidMaxSeenReportIds];
uint8_t seen_report_id_count = 0;

HidByteActivity byte_activity[kHidByteActivitySlots];
uint8_t byte_activity_width = 0;
uint32_t sampled_reports = 0;

void note_byte_activity(uint8_t const *report, uint16_t len) {
    const uint8_t width = len < kHidByteActivitySlots
        ? static_cast<uint8_t>(len)
        : kHidByteActivitySlots;
    for (uint8_t i = 0; i < width; ++i) {
        if (i >= byte_activity_width) {
            byte_activity[i].min_value = report[i];
            byte_activity[i].max_value = report[i];
            byte_activity[i].bits_seen = report[i];
            continue;
        }
        if (report[i] < byte_activity[i].min_value) {
            byte_activity[i].min_value = report[i];
        }
        if (report[i] > byte_activity[i].max_value) {
            byte_activity[i].max_value = report[i];
        }
        byte_activity[i].bits_seen |= report[i];
    }
    if (width > byte_activity_width) {
        byte_activity_width = width;
    }
    sampled_reports++;
}

void note_seen_report_id(uint8_t id) {
    for (uint8_t i = 0; i < seen_report_id_count; ++i) {
        if (seen_report_ids[i] == id) {
            return;
        }
    }
    if (seen_report_id_count < kHidMaxSeenReportIds) {
        seen_report_ids[seen_report_id_count++] = id;
    }
}

// Shortest report that could carry the four axes the fallback expects. Guards
// against latching the fallback onto a short consumer-control or battery report
// that a pad may interleave with its gamepad reports.
constexpr uint16_t kMinAdoptableReportLen = 6;

uint32_t extract_bits(
    uint8_t const *payload,
    uint16_t payload_len,
    uint16_t bit_offset,
    uint8_t bit_size
) {
    if (bit_size == 0 || bit_size > 32) {
        return 0;
    }
    uint32_t result = 0;
    for (uint8_t i = 0; i < bit_size; ++i) {
        const uint16_t bit = static_cast<uint16_t>(bit_offset + i);
        const uint16_t byte_index = static_cast<uint16_t>(bit >> 3);
        if (byte_index >= payload_len) {
            break;
        }
        if ((payload[byte_index] >> (bit & 0x07)) & 0x01) {
            result |= (1u << i);
        }
    }
    return result;
}

// Sign-extends a raw field when its declared logical range goes negative.
int32_t to_logical(uint32_t raw, HidFieldLocation const &field) {
    if (field.logical_min < 0 && field.bit_size < 32) {
        const uint32_t sign_bit = 1u << (field.bit_size - 1);
        if ((raw & sign_bit) != 0) {
            return static_cast<int32_t>(raw) - static_cast<int32_t>(sign_bit << 1);
        }
    }
    return static_cast<int32_t>(raw);
}

// Maps a field onto the 0..255 range BridgeControllerState uses, where 0x80 is
// centre for a stick.
uint8_t scale_axis(uint32_t raw, HidFieldLocation const &field) {
    const int32_t value = to_logical(raw, field);
    int32_t min = field.logical_min;
    int32_t max = field.logical_max;
    if (max <= min) {
        return 0x80;
    }
    int32_t clamped = value;
    if (clamped < min) {
        clamped = min;
    }
    if (clamped > max) {
        clamped = max;
    }
    const int64_t span = static_cast<int64_t>(max) - static_cast<int64_t>(min);
    const int64_t scaled =
        ((static_cast<int64_t>(clamped) - min) * 255 + span / 2) / span;
    return static_cast<uint8_t>(scaled);
}

void apply_hat(uint32_t raw, HidFieldLocation const &field, BridgeControllerState &state) {
    const int32_t value = to_logical(raw, field);
    if (value < field.logical_min || value > field.logical_max) {
        return; // Null state: centred.
    }
    const int32_t direction = value - field.logical_min;
    // 0 = north, then clockwise in 45 degree steps.
    switch (direction) {
        case 0: state.dpad_up = true; break;
        case 1: state.dpad_up = true; state.dpad_right = true; break;
        case 2: state.dpad_right = true; break;
        case 3: state.dpad_down = true; state.dpad_right = true; break;
        case 4: state.dpad_down = true; break;
        case 5: state.dpad_down = true; state.dpad_left = true; break;
        case 6: state.dpad_left = true; break;
        case 7: state.dpad_up = true; state.dpad_left = true; break;
        default: break;
    }
}

} // namespace

void generic_hid_reset() {
    active_layout = kFallbackLayout;
    layout_source = HidLayoutSource::Generic;
    layout_name = "GENERIC GUESS";
    layout_locked = false;
    active_button_map = kAndroidButtonMap;
    active_button_map_count = kAndroidButtonMapCount;
    last_report_len = 0;
    last_button_mask = 0;
    last_hat_value = kHidHatCentred;
    seen_report_id_count = 0;
    byte_activity_width = 0;
    sampled_reports = 0;
}

bool generic_hid_apply_descriptor_layout(HidGamepadLayout const &layout) {
    if (!layout.valid) {
        return false;
    }
    active_layout = layout;
    layout_source = HidLayoutSource::Descriptor;
    layout_name = "REPORT DESCRIPTOR";
    layout_locked = true;
    return true;
}

HidGamepadLayout const &generic_hid_active_layout() {
    return active_layout;
}

HidLayoutSource generic_hid_layout_source() {
    return layout_source;
}

char const *generic_hid_layout_name() {
    return layout_name;
}

bool generic_hid_layout_is_from_descriptor() {
    return layout_source == HidLayoutSource::Descriptor;
}

uint8_t generic_hid_seen_report_ids(uint8_t *out, uint8_t capacity) {
    if (out == nullptr || capacity == 0) {
        return 0;
    }
    const uint8_t count = seen_report_id_count < capacity ? seen_report_id_count : capacity;
    std::memcpy(out, seen_report_ids, count);
    return count;
}

uint8_t generic_hid_byte_activity(HidByteActivity *out, uint8_t capacity) {
    if (out == nullptr || capacity == 0) {
        return 0;
    }
    const uint8_t count = byte_activity_width < capacity ? byte_activity_width : capacity;
    for (uint8_t i = 0; i < count; ++i) {
        out[i] = byte_activity[i];
    }
    return count;
}

uint32_t generic_hid_sampled_reports() {
    return sampled_reports;
}

uint32_t generic_hid_last_button_mask() {
    return last_button_mask;
}

uint16_t generic_hid_last_hat_value() {
    return last_hat_value;
}

uint16_t generic_hid_last_report(uint8_t *out, uint16_t capacity) {
    if (out == nullptr || capacity == 0) {
        return 0;
    }
    const uint16_t len = last_report_len < capacity ? last_report_len : capacity;
    std::memcpy(out, last_report, len);
    return len;
}

bool generic_hid_decode_input_report(
    uint8_t const *report,
    uint16_t len,
    BridgeControllerState &state
) {
    if (report == nullptr || len == 0) {
        return false;
    }

    // Captured before the layout check so the diagnostic dump still shows the
    // bytes when the report is one we do not recognise.
    last_report_len = len < kLastReportCapacity ? len : kLastReportCapacity;
    std::memcpy(last_report, report, last_report_len);
    note_seen_report_id(report[0]);
    note_byte_activity(report, len);

    if (!active_layout.valid) {
        return false;
    }

    // Without a descriptor there is nothing to say which layout the pad is
    // using, and the Stellaris changes shape completely between pairing modes.
    // Match on report ID and length, which together identify it unambiguously;
    // failing that, adopt the report ID onto the generic layout, because
    // guessing it wrong rejects every report and looks like a dead controller.
    if (!layout_locked && len >= kMinAdoptableReportLen) {
        for (uint8_t i = 0; i < kKnownLayoutCount; ++i) {
            if (kKnownLayouts[i].report_id == report[0] && kKnownLayouts[i].report_len == len) {
                active_layout = kKnownLayouts[i].layout;
                active_layout.report_id = report[0];
                active_button_map = kKnownLayouts[i].buttons;
                active_button_map_count = kKnownLayouts[i].button_map_count;
                layout_source = HidLayoutSource::Known;
                layout_name = kKnownLayouts[i].name;
                layout_locked = true;
                break;
            }
        }
        if (!layout_locked) {
            active_layout.report_id = report[0];
            layout_source = HidLayoutSource::Generic;
            layout_name = "GENERIC GUESS";
            layout_locked = true;
        }
    }

    uint8_t const *payload = report;
    uint16_t payload_len = len;
    if (active_layout.uses_report_id) {
        if (report[0] != active_layout.report_id) {
            return false;
        }
        payload = report + 1;
        payload_len = static_cast<uint16_t>(len - 1);
    }
    if (payload_len == 0) {
        return false;
    }

    state = BridgeControllerState{};

    if (active_layout.x.present) {
        state.left_stick_x =
            scale_axis(extract_bits(payload, payload_len, active_layout.x.bit_offset,
                                    active_layout.x.bit_size),
                       active_layout.x);
    }
    if (active_layout.y.present) {
        state.left_stick_y =
            scale_axis(extract_bits(payload, payload_len, active_layout.y.bit_offset,
                                    active_layout.y.bit_size),
                       active_layout.y);
    }

    // Right stick is Z/Rz on almost every Bluetooth pad; Rx/Ry is the fallback
    // for the ones that use the rotation axes for the sticks instead. Whichever
    // pair is not taken by the stick is then available for the triggers.
    const bool right_stick_uses_z = active_layout.z.present && active_layout.rz.present;
    HidFieldLocation const &right_x = right_stick_uses_z ? active_layout.z : active_layout.rx;
    HidFieldLocation const &right_y = right_stick_uses_z ? active_layout.rz : active_layout.ry;
    if (right_x.present) {
        state.right_stick_x = scale_axis(
            extract_bits(payload, payload_len, right_x.bit_offset, right_x.bit_size), right_x);
    }
    if (right_y.present) {
        state.right_stick_y = scale_axis(
            extract_bits(payload, payload_len, right_y.bit_offset, right_y.bit_size), right_y);
    }

    HidFieldLocation const &left_trigger =
        right_stick_uses_z ? active_layout.rx : active_layout.z;
    HidFieldLocation const &right_trigger =
        right_stick_uses_z ? active_layout.ry : active_layout.rz;
    bool analog_triggers = false;
    if (left_trigger.present) {
        state.left_trigger = scale_axis(
            extract_bits(payload, payload_len, left_trigger.bit_offset, left_trigger.bit_size),
            left_trigger);
        analog_triggers = true;
    }
    if (right_trigger.present) {
        state.right_trigger = scale_axis(
            extract_bits(payload, payload_len, right_trigger.bit_offset, right_trigger.bit_size),
            right_trigger);
        analog_triggers = true;
    }

    last_hat_value = kHidHatCentred;
    if (active_layout.hat.present) {
        const uint32_t raw_hat = extract_bits(
            payload, payload_len, active_layout.hat.bit_offset, active_layout.hat.bit_size);
        apply_hat(raw_hat, active_layout.hat, state);
        const int32_t hat_value = to_logical(raw_hat, active_layout.hat);
        if (hat_value >= active_layout.hat.logical_min
            && hat_value <= active_layout.hat.logical_max) {
            last_hat_value = static_cast<uint16_t>(hat_value - active_layout.hat.logical_min);
        }
    }

    last_button_mask = 0;
    if (active_layout.buttons_present) {
        const uint8_t count =
            active_layout.button_count > 32 ? 32 : active_layout.button_count;
        for (uint8_t b = 0; b < count; ++b) {
            const uint16_t bit = static_cast<uint16_t>(active_layout.button_bit_offset + b);
            if (extract_bits(payload, payload_len, bit, 1) != 0) {
                last_button_mask |= (1u << b);
            }
        }
    }

    if (active_layout.buttons_present) {
        for (uint8_t i = 0; i < active_button_map_count; ++i) {
            ButtonBinding const &binding = active_button_map[i];
            if (binding.number > active_layout.button_count) {
                continue;
            }
            const uint16_t bit = static_cast<uint16_t>(
                active_layout.button_bit_offset + (binding.number - 1));
            state.*(binding.field) = extract_bits(payload, payload_len, bit, 1) != 0;
        }
    }

    // A pad with digital shoulder triggers reports them as buttons only. Give
    // the host full deflection so trigger-driven games still work.
    if (!analog_triggers) {
        state.left_trigger = state.l2_pressed ? 0xFF : 0x00;
        state.right_trigger = state.r2_pressed ? 0xFF : 0x00;
    }

    return true;
}
