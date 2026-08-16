#ifndef DS5_BRIDGE_GENERIC_HID_INPUT_DECODER_H
#define DS5_BRIDGE_GENERIC_HID_INPUT_DECODER_H

#include <cstdint>

#include "controller_state.h"
#include "hid_report_descriptor.h"

//
// Decodes a third-party Bluetooth gamepad's input report into the fields the
// XUSB persona consumes.
//
// The layout comes from the pad's own report descriptor (see
// hid_report_descriptor.h). Until that arrives -- or if the SDP query fails --
// a built-in layout covering the most common generic-gamepad report is used, so
// the pad is never completely dead.
//
// Only the 21 BridgeControllerState fields xusb360_persona_encode_input() reads
// are populated. Battery, motion, touch, and audio state stay at their defaults
// because a generic pad reports none of them.
//

// Reverts to the built-in fallback layout and clears the captured report.
void generic_hid_reset();

// Adopts the parsed descriptor layout. No-op unless the layout is valid.
bool generic_hid_apply_descriptor_layout(HidGamepadLayout const &layout);

HidGamepadLayout const &generic_hid_active_layout();

enum class HidLayoutSource : uint8_t {
    // Generic guess: report ID adopted from the wire, offsets assumed.
    Generic,
    // Matched a layout verified against real hardware by report ID and length.
    Known,
    // Parsed from the pad's own report descriptor.
    Descriptor,
};

HidLayoutSource generic_hid_layout_source();
char const *generic_hid_layout_name();

// False while the built-in fallback is in use, i.e. the decode is a guess.
bool generic_hid_layout_is_from_descriptor();

// report[0] is the HID report ID; report + 1 is the payload. Returns false when
// the report does not match the active layout, leaving state untouched.
bool generic_hid_decode_input_report(
    uint8_t const *report,
    uint16_t len,
    BridgeControllerState &state
);

// The most recent report handed to the decoder, for the diagnostic dump.
uint16_t generic_hid_last_report(uint8_t *out, uint16_t capacity);

// Raw button bits and hat position from the most recent report, extracted using
// the active layout but before any naming is applied. The guided capture works
// from these so it can report what the pad actually sent rather than what the
// current button map believes it means.
inline constexpr uint16_t kHidHatCentred = 0xFFFF;
uint32_t generic_hid_last_button_mask();
uint16_t generic_hid_last_hat_value();

// Every distinct report ID the pad has sent, whether or not the active layout
// accepts it. A pad that splits its controls across several reports would
// otherwise look like it simply has no analog triggers, because the decoder
// rejects everything that does not match the layout it locked onto.
inline constexpr uint8_t kHidMaxSeenReportIds = 8;
uint8_t generic_hid_seen_report_ids(uint8_t *out, uint8_t capacity);

// Lowest and highest value every byte position has ever carried.
//
// This is the tool for finding fields nobody told us about. A byte that only
// ever reads 00-01 is a bit field; one that sweeps 00-FF is an axis or a
// trigger; one that never moves is padding. It answers "is there analog data
// anywhere in this report" without having to guess an offset first, and it
// works for any pad in any mode.
inline constexpr uint8_t kHidByteActivitySlots = 24;
struct HidByteActivity {
    uint8_t min_value;
    uint8_t max_value;
    // OR of every value this byte has carried. For a bit field this is the set
    // of bits that exist, which min/max cannot show: pressing buttons one at a
    // time leaves max at the highest single bit, not the union.
    uint8_t bits_seen;
};
uint8_t generic_hid_byte_activity(HidByteActivity *out, uint8_t capacity);
uint32_t generic_hid_sampled_reports();

#endif // DS5_BRIDGE_GENERIC_HID_INPUT_DECODER_H
