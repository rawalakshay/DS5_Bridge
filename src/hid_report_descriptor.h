#ifndef DS5_BRIDGE_HID_REPORT_DESCRIPTOR_H
#define DS5_BRIDGE_HID_REPORT_DESCRIPTOR_H

#include <cstdint>

//
// A remote device's HID report descriptor, and the gamepad layout parsed from
// it.
//
// Stellaris mode has no hardcoded byte layout for the pad it talks to. Instead
// the descriptor the device publishes over SDP (attribute 0x0206,
// HIDDescriptorList) is fetched at connect time and parsed here, so the same
// firmware decodes any standards-compliant Bluetooth gamepad -- including the
// Stellaris in each of its four pairing modes, which do not share a report
// format.
//
// Parsing is pure: no I/O, no BTstack, no Pico SDK. tests/firmware exercises it
// directly against synthetic descriptors.
//

inline constexpr uint16_t kHidReportDescriptorMaxBytes = 1024;
inline constexpr uint8_t kHidMaxButtons = 32;

// Where one field lives inside a report, and the range it reports over.
// bit_offset is relative to the start of the report payload, i.e. after the
// report ID byte when the device uses report IDs.
struct HidFieldLocation {
    bool present;
    uint16_t bit_offset;
    uint8_t bit_size;
    int32_t logical_min;
    int32_t logical_max;
};

struct HidGamepadLayout {
    bool valid;
    // True when the device prefixes reports with a report ID byte.
    bool uses_report_id;
    uint8_t report_id;
    // Payload size, excluding the report ID byte.
    uint16_t report_bits;

    HidFieldLocation x;
    HidFieldLocation y;
    HidFieldLocation z;
    HidFieldLocation rx;
    HidFieldLocation ry;
    HidFieldLocation rz;
    HidFieldLocation hat;

    bool buttons_present;
    uint16_t button_bit_offset;
    uint8_t button_count;
};

// How the descriptor fetch went. Reported by the diagnostic dump, because a
// failed fetch is otherwise indistinguishable from a pad that publishes
// nothing useful.
enum HidDescriptorFetchStatus : uint8_t {
    HidDescriptorFetchIdle,        // No pad connected yet.
    HidDescriptorFetchPending,     // Query queued or in flight.
    HidDescriptorFetchQueryFailed, // BTstack refused to start it; detail = status.
    HidDescriptorFetchNoData,      // Query ran, but no HID descriptor came back.
    HidDescriptorFetchParseFailed, // Descriptor arrived but held no gamepad.
    HidDescriptorFetchOk,
};

void hid_report_descriptor_set_status(HidDescriptorFetchStatus status, uint8_t detail);
HidDescriptorFetchStatus hid_report_descriptor_status();
uint8_t hid_report_descriptor_status_detail();

// Incremental storage, filled a byte at a time from the SDP attribute stream.
void hid_report_descriptor_reset();
bool hid_report_descriptor_store_byte(uint8_t byte);
uint16_t hid_report_descriptor_length();

// Parses whatever has been stored. Returns true when a usable gamepad layout
// was found; the result is available from hid_report_descriptor_layout().
bool hid_report_descriptor_parse();
HidGamepadLayout const &hid_report_descriptor_layout();

// Parses from caller-supplied memory into a caller-supplied layout. The
// firmware path goes through the functions above; this is the seam the tests
// drive.
bool hid_report_descriptor_parse_buffer(
    uint8_t const *data,
    uint16_t len,
    HidGamepadLayout &layout
);

#endif // DS5_BRIDGE_HID_REPORT_DESCRIPTOR_H
