//
// Keyboard-typed diagnostic dump. See stellaris_diagnostics.h.
//

#include "stellaris_diagnostics.h"

#if STELLARIS_ONLY

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "bridge_latency.h"
#include "bt.h"
#include "generic_hid_input_decoder.h"
#include "hid_report_descriptor.h"
#include "persona/host_persona.h"
#include "tusb.h"

namespace {

// Sized for the header, which is the longest block: the byte-range table alone
// is up to 24 entries. append() truncates rather than overruns, but a truncated
// diagnostic is a wasted round trip.
constexpr uint16_t kTextCapacity = 1280;
// One key event per tick at this spacing. Press and release are separate
// events, so a character costs twice this. Slow enough that a text editor keeps
// up, fast enough that the whole dump lands in a couple of seconds.
// One key event per tick. Was 6 ms, which occasionally lost a character across
// a shift boundary -- "3F" arriving as "#" -- because the modifier from one
// keystroke bled into the next. The dump is short; reliability is worth more
// than the extra second.
constexpr uint32_t kKeyIntervalMs = 10;

constexpr uint8_t kModifierLeftShift = 0x02;

char text[kTextCapacity];
uint16_t text_len = 0;
uint16_t text_cursor = 0;
bool key_is_down = false;
uint32_t next_event_ms = 0;

// After the header, the dump keeps typing a line every time the pad's report
// changes. That turns mapping buttons from "one dump per button" into a single
// pass: hold the editor open, press each control in turn, read the transitions.
enum class DumpState : uint8_t {
    Idle,
    TypingHeader,
    Guided,
    RumbleProbe,
};

DumpState state = DumpState::Idle;

//
// Guided capture.
//
// Free-form capture worked but left one thing inferred: which physical button
// each report bit belongs to. Reading that out of an unprompted press order is
// guesswork. Naming the button first and recording what arrives removes it.
//
enum class GuidedKind : uint8_t {
    Button,
    Hat,
    // Dumps the whole report instead of naming a control. Used for the
    // triggers: a half-pressed analog trigger puts a mid-range value somewhere
    // in the report, and a purely digital one does not. Nothing else
    // distinguishes the two from the outside.
    RawSample,
};

enum class GuidedPhase : uint8_t {
    Prompt,      // Typing "PRESS X".
    WaitPress,   // Waiting for the pad.
    Result,      // Typing what arrived.
    WaitRelease, // Waiting for the control to come back up.
};

struct GuidedStep {
    char const *label;
    GuidedKind kind;
};

constexpr GuidedStep kGuidedSteps[] = {
    {"A", GuidedKind::Button},
    {"B", GuidedKind::Button},
    {"X", GuidedKind::Button},
    {"Y", GuidedKind::Button},
    {"L1", GuidedKind::Button},
    {"R1", GuidedKind::Button},
    {"L2", GuidedKind::Button},
    {"R2", GuidedKind::Button},
    {"SELECT", GuidedKind::Button},
    {"START", GuidedKind::Button},
    {"L3 (CLICK LEFT STICK)", GuidedKind::Button},
    {"R3 (CLICK RIGHT STICK)", GuidedKind::Button},
    // HOME is deliberately absent. The bridge presents as an Xbox pad, so the
    // Guide button opens the Game Bar and takes focus away from the editor --
    // every step after it would type into the overlay and be lost. It is
    // already mapped to button 13 from an earlier capture and pinned in tests,
    // so there is nothing to gain by asking for it again.
    {"DPAD UP", GuidedKind::Hat},
    {"DPAD RIGHT", GuidedKind::Hat},
    {"DPAD DOWN", GuidedKind::Hat},
    {"DPAD LEFT", GuidedKind::Hat},
    {"L2 HALFWAY AND HOLD", GuidedKind::RawSample},
    {"R2 HALFWAY AND HOLD", GuidedKind::RawSample},
};

constexpr uint8_t kGuidedStepCount =
    static_cast<uint8_t>(sizeof(kGuidedSteps) / sizeof(kGuidedSteps[0]));

// A pad need not have every control -- HOME in particular is often absent -- so
// a step that goes unanswered is skipped rather than blocking the run.
constexpr uint32_t kGuidedStepTimeoutMs = 15000;

uint8_t guided_index = 0;
GuidedPhase guided_phase = GuidedPhase::Prompt;
uint32_t guided_deadline_ms = 0;

//
// Rumble probe.
//
// A generic pad publishes no output-report format, and this one publishes no
// SDP record at all, so there is nothing to read. The only detector available
// is the person holding the pad: each candidate is announced in text, then
// driven at full scale for a moment. Whichever one buzzes is the answer.
//
struct RumbleCandidate {
    char const *label;
    uint8_t len;
    uint8_t on[10];
    uint8_t off[10];
};

constexpr RumbleCandidate kRumbleCandidates[] = {
    {"01 TWO-BYTE", 3, {0x01, 0xFF, 0xFF}, {0x01, 0x00, 0x00}},
    {"02 TWO-BYTE", 3, {0x02, 0xFF, 0xFF}, {0x02, 0x00, 0x00}},
    {"03 TWO-BYTE", 3, {0x03, 0xFF, 0xFF}, {0x03, 0x00, 0x00}},
    {"05 TWO-BYTE", 3, {0x05, 0xFF, 0xFF}, {0x05, 0x00, 0x00}},
    {"07 TWO-BYTE", 3, {0x07, 0xFF, 0xFF}, {0x07, 0x00, 0x00}},
    {"01 FOUR-BYTE", 5, {0x01, 0x00, 0xFF, 0xFF, 0x00}, {0x01, 0x00, 0x00, 0x00, 0x00}},
    {"01 XINPUT-LIKE", 8,
     {0x01, 0x08, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00},
     {0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {"00 XBOX-LIKE", 8,
     {0x00, 0x08, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00},
     {0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {"10 SWITCH-LIKE", 10,
     {0x10, 0x00, 0x74, 0xBE, 0xBD, 0x6F, 0x74, 0xBE, 0xBD, 0x6F},
     {0x10, 0x01, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40}},
    {"11 DS4-LIKE", 8,
     {0x11, 0xC0, 0x20, 0xF3, 0x04, 0x00, 0xFF, 0xFF},
     {0x11, 0xC0, 0x20, 0xF3, 0x04, 0x00, 0x00, 0x00}},
};

constexpr uint8_t kRumbleCandidateCount =
    static_cast<uint8_t>(sizeof(kRumbleCandidates) / sizeof(kRumbleCandidates[0]));
constexpr uint32_t kRumbleOnMs = 700;
constexpr uint32_t kRumbleGapMs = 500;

uint8_t rumble_index = 0;
bool rumble_driving = false;
uint32_t rumble_until_ms = 0;

// ASCII to HID usage. Returns false for anything not worth typing, which the
// caller skips.
bool usage_for_char(char c, uint8_t &usage, uint8_t &modifier) {
    modifier = 0;
    if (c >= 'a' && c <= 'z') {
        usage = static_cast<uint8_t>(0x04 + (c - 'a'));
        return true;
    }
    if (c >= 'A' && c <= 'Z') {
        usage = static_cast<uint8_t>(0x04 + (c - 'A'));
        modifier = kModifierLeftShift;
        return true;
    }
    if (c >= '1' && c <= '9') {
        usage = static_cast<uint8_t>(0x1E + (c - '1'));
        return true;
    }
    switch (c) {
        case '0': usage = 0x27; return true;
        case '\n': usage = 0x28; return true;
        case ' ': usage = 0x2C; return true;
        case '-': usage = 0x2D; return true;
        case '=': usage = 0x2E; return true;
        case '/': usage = 0x38; return true;
        case '.': usage = 0x37; return true;
        case ',': usage = 0x36; return true;
        case ';': usage = 0x33; return true;
        case ':': usage = 0x33; modifier = kModifierLeftShift; return true;
        case '(': usage = 0x26; modifier = kModifierLeftShift; return true;
        case ')': usage = 0x27; modifier = kModifierLeftShift; return true;
        case '?': usage = 0x38; modifier = kModifierLeftShift; return true;
        case '*': usage = 0x25; modifier = kModifierLeftShift; return true;
        default: return false;
    }
}

void append(char const *fmt, ...) __attribute__((format(printf, 1, 2)));

void append(char const *fmt, ...) {
    if (text_len >= kTextCapacity - 1) {
        return;
    }
    va_list args;
    va_start(args, fmt);
    const int written = vsnprintf(
        text + text_len,
        static_cast<size_t>(kTextCapacity - text_len),
        fmt,
        args
    );
    va_end(args);
    if (written > 0) {
        const uint16_t room = static_cast<uint16_t>(kTextCapacity - 1 - text_len);
        text_len = static_cast<uint16_t>(
            text_len + (static_cast<uint16_t>(written) > room ? room : written)
        );
    }
}

void append_field(char const *name, HidFieldLocation const &field) {
    if (!field.present) {
        append("%s -  ", name);
        return;
    }
    append("%s %u/%u  ", name, (unsigned int) field.bit_offset, (unsigned int) field.bit_size);
}

char const *fetch_status_name(HidDescriptorFetchStatus status) {
    switch (status) {
        case HidDescriptorFetchIdle: return "NO PAD CONNECTED";
        case HidDescriptorFetchPending: return "STILL RUNNING";
        case HidDescriptorFetchQueryFailed: return "QUERY REFUSED";
        case HidDescriptorFetchNoData: return "PAD SENT NOTHING";
        case HidDescriptorFetchParseFailed: return "NO GAMEPAD IN DESCRIPTOR";
        case HidDescriptorFetchOk: return "OK";
        default: return "UNKNOWN";
    }
}

void build_text() {
    text_len = 0;
    text_cursor = 0;

    HidGamepadLayout const &layout = generic_hid_active_layout();
    const uint16_t descriptor_len = hid_report_descriptor_length();

    append("\n--- DS5 BRIDGE STELLARIS ---\n");
    append(
        "FW %u.%u.%u\n",
        (unsigned int) DS5_FIRMWARE_VERSION_MAJOR,
        (unsigned int) DS5_FIRMWARE_VERSION_MINOR,
        (unsigned int) DS5_FIRMWARE_VERSION_PATCH
    );
    append(
        "SDP %s (%u BYTES, CODE %u)\n",
        fetch_status_name(hid_report_descriptor_status()),
        (unsigned int) descriptor_len,
        (unsigned int) hid_report_descriptor_status_detail()
    );
    append("LAYOUT %s\n", generic_hid_layout_name());
    append(
        "REPORT ID %u  PAYLOAD %u BITS\n",
        (unsigned int) (layout.uses_report_id ? layout.report_id : 0),
        (unsigned int) layout.report_bits
    );

    append("AXES ");
    append_field("X", layout.x);
    append_field("Y", layout.y);
    append("\n     ");
    append_field("Z", layout.z);
    append_field("RZ", layout.rz);
    append("\n     ");
    append_field("RX", layout.rx);
    append_field("RY", layout.ry);
    append_field("HAT", layout.hat);
    append("\n");

    if (layout.buttons_present) {
        append(
            "BUTTONS %u AT BIT %u\n",
            (unsigned int) layout.button_count,
            (unsigned int) layout.button_bit_offset
        );
    } else {
        append("BUTTONS NONE\n");
    }

    uint8_t last[24];
    const uint16_t last_len = generic_hid_last_report(last, sizeof(last));
    // More than one ID here means the pad splits its controls across several
    // reports, and the decoder is only reading one of them.
    uint8_t ids[kHidMaxSeenReportIds];
    const uint8_t id_count = generic_hid_seen_report_ids(ids, sizeof(ids));
    append("REPORT IDS SEEN %u:", (unsigned int) id_count);
    for (uint8_t i = 0; i < id_count; ++i) {
        append(" %02X", (unsigned int) ids[i]);
    }
    append("\n");

    BtDeviceIdentitySnapshot identity{};
    if (bt_get_device_identity(&identity)) {
        append("PAD %s BONDED %u\n", identity.address, (unsigned int) identity.link_key_known);
    }
    if (bt_has_signal_strength()) {
        const int8_t rssi = bt_get_signal_strength();
        append("RSSI %s%u DBM\n", rssi < 0 ? "-" : "", (unsigned int) (rssi < 0 ? -rssi : rssi));
    }

    // The field finder. A byte that sweeps a wide range is an axis or an analog
    // trigger; a narrow range is a bit field; MIN == MAX never moved at all.
    // Reading this beats guessing offsets, and it needs no prior knowledge of
    // the pad.
    HidByteActivity activity[kHidByteActivitySlots];
    const uint8_t width = generic_hid_byte_activity(activity, sizeof(activity) / sizeof(activity[0]));
    if (width > 0) {
        append("BYTES OVER %u REPORTS (MIN-MAX/BITS)\n",
               (unsigned int) generic_hid_sampled_reports());
        for (uint8_t i = 0; i < width; ++i) {
            const bool moved = activity[i].min_value != activity[i].max_value;
            append(
                " %u:%02X-%02X/%02X%s",
                (unsigned int) i,
                (unsigned int) activity[i].min_value,
                (unsigned int) activity[i].max_value,
                (unsigned int) activity[i].bits_seen,
                moved ? "*" : " "
            );
            if ((i % 3) == 2) {
                append("\n");
            }
        }
        if ((width % 3) != 0) {
            append("\n");
        }
        append("STAR MEANS MOVED. FULL 00-FF RANGE MEANS AXIS OR TRIGGER\n");
    }

    append("LAST REPORT ");
    if (last_len == 0) {
        append("NONE - NO INPUT RECEIVED");
    } else {
        for (uint16_t i = 0; i < last_len; ++i) {
            append("%02X ", (unsigned int) last[i]);
        }
    }
    // Timing over a handful of reports is dominated by connect-time transients:
    // USB is still attaching and input is held quiet, so the first send lands
    // hundreds of milliseconds after its report. Refuse to print a number that
    // would only mislead.
    constexpr uint32_t kMinTimingSamples = 200;
    const BridgeLatencyStats stats = bridge_latency_snapshot();
    if (stats.interval_samples < kMinTimingSamples) {
        append(
            "TIMING %u/%u SAMPLES - USE THE PAD FOR 30S FIRST\n",
            (unsigned int) stats.interval_samples,
            (unsigned int) kMinTimingSamples
        );
    } else if (stats.interval_samples > 0) {
        const uint32_t avg_us =
            static_cast<uint32_t>(stats.interval_sum_us / stats.interval_samples);
        // Rounded rate from the mean interval. Integer maths only; there is no
        // floating point worth spending on a diagnostic line.
        const uint32_t hz = avg_us > 0 ? (1000000u + avg_us / 2) / avg_us : 0;
        append(
            "LINK %u REPORTS, %u HZ, %u GAPS\n",
            (unsigned int) stats.reports,
            (unsigned int) hz,
            (unsigned int) stats.idle_gaps
        );
        append(
            "INTERVAL MIN %u AVG %u MAX %u US\n",
            (unsigned int) stats.interval_min_us,
            (unsigned int) avg_us,
            (unsigned int) stats.interval_max_us
        );
    }
    if (stats.sends > 0 && stats.interval_samples >= kMinTimingSamples) {
        append(
            "BRIDGE ADD MIN %u AVG %u MAX %u US\n",
            (unsigned int) stats.add_min_us,
            (unsigned int) (stats.add_sum_us / stats.sends),
            (unsigned int) stats.add_max_us
        );
    }

    append("\nGUIDED CAPTURE - PRESS WHAT IT ASKS FOR\n");
    append("DOUBLE PRESS BOOTSEL TO SKIP TO RUMBLE TEST\n");
}

void build_rumble_label(uint8_t send_status) {
    text_len = 0;
    text_cursor = 0;
    RumbleCandidate const &candidate = kRumbleCandidates[rumble_index];
    append("RUMBLE %u OF %u %s: ",
           (unsigned int) (rumble_index + 1),
           (unsigned int) kRumbleCandidateCount,
           candidate.label);
    for (uint8_t i = 0; i < candidate.len; ++i) {
        append("%02X ", (unsigned int) candidate.on[i]);
    }
    // Without this a candidate that never reached the radio is indistinguishable
    // from one the pad received and ignored.
    append("%s\n", send_status == kBtRawOutputSent ? "SENT" : "NOT SENT");
}

void build_guided_prompt() {
    text_len = 0;
    text_cursor = 0;
    append(
        "%u/%u PRESS %s\n",
        (unsigned int) (guided_index + 1),
        (unsigned int) kGuidedStepCount,
        kGuidedSteps[guided_index].label
    );
}

// Which report byte and bit a button number lands on, so the result reads
// against the raw bytes rather than only against a button index.
void append_button_location(uint8_t button_number) {
    HidGamepadLayout const &layout = generic_hid_active_layout();
    const uint16_t payload_bit =
        static_cast<uint16_t>(layout.button_bit_offset + (button_number - 1));
    const uint16_t report_byte =
        static_cast<uint16_t>((payload_bit / 8) + (layout.uses_report_id ? 1 : 0));
    append(" (BYTE %u BIT %u)", (unsigned int) report_byte, (unsigned int) (payload_bit % 8));
}

void build_guided_result(bool timed_out, uint32_t mask, uint16_t hat) {
    text_len = 0;
    text_cursor = 0;
    append("  %s = ", kGuidedSteps[guided_index].label);

    if (timed_out) {
        append("NOT PRESENT (SKIPPED)\n");
        return;
    }

    if (kGuidedSteps[guided_index].kind == GuidedKind::Hat) {
        append("HAT %u\n", (unsigned int) hat);
        return;
    }

    if (kGuidedSteps[guided_index].kind == GuidedKind::RawSample) {
        uint8_t raw[24];
        const uint16_t raw_len = generic_hid_last_report(raw, sizeof(raw));
        for (uint16_t i = 0; i < raw_len; ++i) {
            append("%02X ", (unsigned int) raw[i]);
        }
        append("\n");
        return;
    }

    // Lowest set bit is the answer; anything else means two controls moved
    // together and the reading should not be trusted silently.
    uint8_t number = 0;
    for (uint8_t b = 0; b < 32; ++b) {
        if ((mask & (1u << b)) != 0) {
            number = static_cast<uint8_t>(b + 1);
            break;
        }
    }
    append("BTN %u", (unsigned int) number);
    append_button_location(number);
    if ((mask & (mask - 1)) != 0) {
        append(" WARNING MASK %08X", (unsigned int) mask);
    }
    append("\n");
}

} // namespace

void stellaris_diagnostics_request_dump() {
    // Three-state cycle on the one free gesture: dump and capture input, then
    // probe for a working rumble format, then stop. Stopping matters -- neither
    // phase should be left running into whatever window gets focus next.
    if (state == DumpState::TypingHeader || state == DumpState::Guided) {
        state = DumpState::RumbleProbe;
        rumble_index = 0;
        rumble_driving = false;
        rumble_until_ms = 0;
        // Let the poll drive the first candidate, so it goes through the same
        // send-then-announce path as the rest.
        text_len = 0;
        text_cursor = 0;
        key_is_down = false;
        next_event_ms = 0;
        return;
    }
    if (state == DumpState::RumbleProbe) {
        (void)bt_send_raw_hid_output(
            kRumbleCandidates[rumble_index].off, kRumbleCandidates[rumble_index].len);
        state = DumpState::Idle;
        text_len = 0;
        text_cursor = 0;
        return;
    }

    build_text();
    if (text_len == 0) {
        return;
    }
    state = DumpState::TypingHeader;
    guided_index = 0;
    guided_phase = GuidedPhase::Prompt;
    guided_deadline_ms = 0;
    key_is_down = false;
    next_event_ms = 0;
}

void stellaris_diagnostics_poll(uint32_t now_ms) {
    if (state == DumpState::Idle) {
        return;
    }
    if (next_event_ms != 0 && static_cast<int32_t>(now_ms - next_event_ms) < 0) {
        return;
    }

    const uint8_t instance = host_persona_keyboard_hid_instance();
    if (!tud_hid_n_ready(instance)) {
        return;
    }

    uint8_t report[8]{};

    if (key_is_down) {
        // Release. An all-zero report is what tells the host the key came up.
        if (tud_hid_n_report(instance, 0, report, sizeof(report))) {
            key_is_down = false;
            next_event_ms = now_ms + kKeyIntervalMs;
        }
        return;
    }

    // Skip anything the scancode table does not cover rather than stalling.
    uint8_t usage = 0;
    uint8_t modifier = 0;
    while (text_cursor < text_len && !usage_for_char(text[text_cursor], usage, modifier)) {
        text_cursor++;
    }

    if (text_cursor >= text_len) {
        if (state == DumpState::RumbleProbe) {
            RumbleCandidate const &candidate = kRumbleCandidates[rumble_index];
            if (!rumble_driving) {
                // Send first, then announce it with the result, so the pad is
                // already buzzing while the label appears.
                const uint8_t status = bt_send_raw_hid_output(candidate.on, candidate.len);
                if (status == kBtRawOutputBusy) {
                    // Never advance on a busy channel; that would burn a
                    // candidate without testing it.
                    next_event_ms = now_ms + 10;
                    return;
                }
                rumble_driving = true;
                rumble_until_ms = now_ms + kRumbleOnMs;
                build_rumble_label(status);
                return;
            }
            if (static_cast<int32_t>(now_ms - rumble_until_ms) < 0) {
                return;
            }
            (void)bt_send_raw_hid_output(candidate.off, candidate.len);
            rumble_driving = false;
            rumble_index++;
            if (rumble_index >= kRumbleCandidateCount) {
                state = DumpState::Idle;
                return;
            }
            next_event_ms = now_ms + kRumbleGapMs;
            return;
        }

        // Header finished: begin the guided run. The first prompt has to be
        // built here -- falling straight through to the Prompt phase would put
        // the run into WaitPress with nothing on screen, so step 1 would be
        // silently unprompted.
        if (state == DumpState::TypingHeader) {
            state = DumpState::Guided;
            guided_index = 0;
            guided_phase = GuidedPhase::Prompt;
            build_guided_prompt();
            return;
        }

        const uint32_t mask = generic_hid_last_button_mask();
        const uint16_t hat = generic_hid_last_hat_value();
        const GuidedKind kind = kGuidedSteps[guided_index].kind;
        const bool control_down = kind == GuidedKind::Hat ? (hat != kHidHatCentred) : (mask != 0);

        switch (guided_phase) {
            case GuidedPhase::Prompt:
                // Prompt has finished typing. Require the pad to be at rest
                // first, so a control still held from the previous step is not
                // read as the answer to this one.
                if (control_down) {
                    return;
                }
                guided_phase = GuidedPhase::WaitPress;
                guided_deadline_ms = now_ms + kGuidedStepTimeoutMs;
                return;

            case GuidedPhase::WaitPress:
                if (control_down) {
                    build_guided_result(false, mask, hat);
                    guided_phase = GuidedPhase::Result;
                    return;
                }
                if (static_cast<int32_t>(now_ms - guided_deadline_ms) >= 0) {
                    build_guided_result(true, 0, kHidHatCentred);
                    guided_phase = GuidedPhase::Result;
                }
                return;

            case GuidedPhase::Result:
                guided_phase = GuidedPhase::WaitRelease;
                return;

            case GuidedPhase::WaitRelease:
                if (control_down) {
                    return;
                }
                guided_index++;
                if (guided_index >= kGuidedStepCount) {
                    append("GUIDED CAPTURE DONE\n");
                    state = DumpState::Idle;
                    return;
                }
                build_guided_prompt();
                guided_phase = GuidedPhase::Prompt;
                return;
        }
        return;
    }

    report[0] = modifier;
    report[2] = usage;
    if (tud_hid_n_report(instance, 0, report, sizeof(report))) {
        text_cursor++;
        key_is_down = true;
        next_event_ms = now_ms + kKeyIntervalMs;
    }
}

#else // STELLARIS_ONLY

void stellaris_diagnostics_request_dump() {}

void stellaris_diagnostics_poll(uint32_t now_ms) {
    (void) now_ms;
}

#endif // STELLARIS_ONLY
