//
// HID report descriptor parsing. See hid_report_descriptor.h for why this
// exists.
//

#include "hid_report_descriptor.h"

#include <cstring>

namespace {

// Item prefix decoding, USB HID 1.11 section 6.2.2.
constexpr uint8_t kItemTypeMain = 0;
constexpr uint8_t kItemTypeGlobal = 1;
constexpr uint8_t kItemTypeLocal = 2;

constexpr uint8_t kMainTagInput = 0x8;
constexpr uint8_t kMainTagCollection = 0xA;
constexpr uint8_t kMainTagEndCollection = 0xC;

constexpr uint8_t kGlobalTagUsagePage = 0x0;
constexpr uint8_t kGlobalTagLogicalMin = 0x1;
constexpr uint8_t kGlobalTagLogicalMax = 0x2;
constexpr uint8_t kGlobalTagReportSize = 0x7;
constexpr uint8_t kGlobalTagReportId = 0x8;
constexpr uint8_t kGlobalTagReportCount = 0x9;
constexpr uint8_t kGlobalTagPush = 0xA;
constexpr uint8_t kGlobalTagPop = 0xB;

constexpr uint8_t kLocalTagUsage = 0x0;
constexpr uint8_t kLocalTagUsageMin = 0x1;
constexpr uint8_t kLocalTagUsageMax = 0x2;

constexpr uint16_t kUsagePageGenericDesktop = 0x01;
constexpr uint16_t kUsagePageButton = 0x09;

constexpr uint16_t kUsageX = 0x30;
constexpr uint16_t kUsageY = 0x31;
constexpr uint16_t kUsageZ = 0x32;
constexpr uint16_t kUsageRx = 0x33;
constexpr uint16_t kUsageRy = 0x34;
constexpr uint16_t kUsageRz = 0x35;
constexpr uint16_t kUsageHatSwitch = 0x39;

// A pad rarely exposes more than a couple of input reports, but consumer-control
// and vendor reports alongside the gamepad one are common. Track a handful and
// pick the most gamepad-like at the end.
constexpr uint8_t kMaxTrackedReports = 8;
constexpr uint8_t kMaxLocalUsages = 48;
constexpr uint8_t kMaxGlobalStack = 4;

struct GlobalState {
    uint16_t usage_page;
    int32_t logical_min;
    int32_t logical_max;
    uint8_t report_size;
    uint8_t report_count;
};

struct ReportAccumulator {
    bool used;
    uint8_t report_id;
    uint16_t bit_cursor;
    HidGamepadLayout layout;
};

uint8_t descriptor_storage[kHidReportDescriptorMaxBytes];
uint16_t descriptor_length = 0;
HidGamepadLayout parsed_layout{};
HidDescriptorFetchStatus fetch_status = HidDescriptorFetchIdle;
uint8_t fetch_status_detail = 0;

HidFieldLocation *field_for_usage(HidGamepadLayout &layout, uint16_t usage) {
    switch (usage) {
        case kUsageX: return &layout.x;
        case kUsageY: return &layout.y;
        case kUsageZ: return &layout.z;
        case kUsageRx: return &layout.rx;
        case kUsageRy: return &layout.ry;
        case kUsageRz: return &layout.rz;
        case kUsageHatSwitch: return &layout.hat;
        default: return nullptr;
    }
}

// How gamepad-like a report is. Used only to choose between reports; the exact
// weighting does not matter beyond preferring sticks and buttons over a stray
// consumer-control report that happens to declare one axis.
uint8_t layout_score(HidGamepadLayout const &layout) {
    uint8_t score = 0;
    if (layout.x.present) score += 2;
    if (layout.y.present) score += 2;
    if (layout.z.present) score += 1;
    if (layout.rx.present) score += 1;
    if (layout.ry.present) score += 1;
    if (layout.rz.present) score += 1;
    if (layout.hat.present) score += 2;
    if (layout.buttons_present) score += 4;
    return score;
}

ReportAccumulator *accumulator_for(
    ReportAccumulator *reports,
    uint8_t report_id,
    bool uses_report_id
) {
    for (uint8_t i = 0; i < kMaxTrackedReports; ++i) {
        if (reports[i].used && reports[i].report_id == report_id) {
            return &reports[i];
        }
    }
    for (uint8_t i = 0; i < kMaxTrackedReports; ++i) {
        if (!reports[i].used) {
            reports[i].used = true;
            reports[i].report_id = report_id;
            reports[i].bit_cursor = 0;
            reports[i].layout = HidGamepadLayout{};
            reports[i].layout.uses_report_id = uses_report_id;
            reports[i].layout.report_id = report_id;
            return &reports[i];
        }
    }
    return nullptr;
}

} // namespace

void hid_report_descriptor_set_status(HidDescriptorFetchStatus status, uint8_t detail) {
    fetch_status = status;
    fetch_status_detail = detail;
}

HidDescriptorFetchStatus hid_report_descriptor_status() {
    return fetch_status;
}

uint8_t hid_report_descriptor_status_detail() {
    return fetch_status_detail;
}

void hid_report_descriptor_reset() {
    descriptor_length = 0;
    parsed_layout = HidGamepadLayout{};
}

bool hid_report_descriptor_store_byte(uint8_t byte) {
    if (descriptor_length >= kHidReportDescriptorMaxBytes) {
        return false;
    }
    descriptor_storage[descriptor_length++] = byte;
    return true;
}

uint16_t hid_report_descriptor_length() {
    return descriptor_length;
}

bool hid_report_descriptor_parse() {
    return hid_report_descriptor_parse_buffer(
        descriptor_storage,
        descriptor_length,
        parsed_layout
    );
}

HidGamepadLayout const &hid_report_descriptor_layout() {
    return parsed_layout;
}

bool hid_report_descriptor_parse_buffer(
    uint8_t const *data,
    uint16_t len,
    HidGamepadLayout &layout
) {
    layout = HidGamepadLayout{};
    if (data == nullptr || len == 0) {
        return false;
    }

    GlobalState global{};
    GlobalState global_stack[kMaxGlobalStack]{};
    uint8_t global_stack_depth = 0;

    uint16_t local_usages[kMaxLocalUsages]{};
    uint8_t local_usage_count = 0;
    uint32_t usage_min = 0;
    uint32_t usage_max = 0;
    bool usage_range_valid = false;

    uint8_t report_id = 0;
    bool uses_report_id = false;

    ReportAccumulator reports[kMaxTrackedReports]{};

    uint16_t i = 0;
    while (i < len) {
        const uint8_t prefix = data[i++];

        // Long items carry no information we need; skip them wholesale.
        if (prefix == 0xFE) {
            if (i + 1 >= len) {
                break;
            }
            const uint8_t long_size = data[i];
            i = static_cast<uint16_t>(i + 2 + long_size);
            continue;
        }

        const uint8_t tag = static_cast<uint8_t>((prefix >> 4) & 0x0F);
        const uint8_t type = static_cast<uint8_t>((prefix >> 2) & 0x03);
        uint8_t size = static_cast<uint8_t>(prefix & 0x03);
        if (size == 3) {
            size = 4;
        }
        if (static_cast<uint32_t>(i) + size > len) {
            break;
        }

        uint32_t value = 0;
        for (uint8_t b = 0; b < size; ++b) {
            value |= static_cast<uint32_t>(data[i + b]) << (8u * b);
        }
        i = static_cast<uint16_t>(i + size);

        // Logical minimum is signed; the rest are read unsigned. Sign-extend
        // from the item's own width.
        int32_t signed_value = static_cast<int32_t>(value);
        if (size == 1 && (value & 0x80u) != 0) {
            signed_value = static_cast<int32_t>(value) - 0x100;
        } else if (size == 2 && (value & 0x8000u) != 0) {
            signed_value = static_cast<int32_t>(value) - 0x10000;
        }

        switch (type) {
            case kItemTypeGlobal:
                switch (tag) {
                    case kGlobalTagUsagePage:
                        global.usage_page = static_cast<uint16_t>(value);
                        break;
                    case kGlobalTagLogicalMin:
                        global.logical_min = signed_value;
                        break;
                    case kGlobalTagLogicalMax:
                        // Logical maximum is signed too, but a descriptor that
                        // declares an 8-bit axis as 0..255 writes 0xFF, which
                        // sign-extends to -1. Treat a negative maximum below the
                        // minimum as unsigned, which is what every real device
                        // means.
                        global.logical_max =
                            (signed_value < global.logical_min)
                                ? static_cast<int32_t>(value)
                                : signed_value;
                        break;
                    case kGlobalTagReportSize:
                        global.report_size = static_cast<uint8_t>(value);
                        break;
                    case kGlobalTagReportCount:
                        global.report_count = static_cast<uint8_t>(value);
                        break;
                    case kGlobalTagReportId:
                        report_id = static_cast<uint8_t>(value);
                        uses_report_id = true;
                        break;
                    case kGlobalTagPush:
                        if (global_stack_depth < kMaxGlobalStack) {
                            global_stack[global_stack_depth++] = global;
                        }
                        break;
                    case kGlobalTagPop:
                        if (global_stack_depth > 0) {
                            global = global_stack[--global_stack_depth];
                        }
                        break;
                    default:
                        break;
                }
                break;

            case kItemTypeLocal:
                switch (tag) {
                    case kLocalTagUsage:
                        if (local_usage_count < kMaxLocalUsages) {
                            // A 4-byte usage carries the page in its high half.
                            local_usages[local_usage_count++] =
                                (size == 4)
                                    ? static_cast<uint16_t>(value & 0xFFFFu)
                                    : static_cast<uint16_t>(value);
                        }
                        break;
                    case kLocalTagUsageMin:
                        usage_min = (size == 4) ? (value & 0xFFFFu) : value;
                        usage_range_valid = true;
                        break;
                    case kLocalTagUsageMax:
                        usage_max = (size == 4) ? (value & 0xFFFFu) : value;
                        break;
                    default:
                        break;
                }
                break;

            case kItemTypeMain: {
                if (tag == kMainTagInput) {
                    ReportAccumulator *acc =
                        accumulator_for(reports, report_id, uses_report_id);
                    if (acc != nullptr && global.report_size > 0) {
                        const uint16_t field_bits = static_cast<uint16_t>(
                            static_cast<uint16_t>(global.report_size) * global.report_count
                        );
                        // Bit 0 of an Input item marks constant data: padding,
                        // with no usage attached.
                        const bool is_constant = (value & 0x01u) != 0;

                        if (!is_constant) {
                            if (
                                global.usage_page == kUsagePageButton
                                && usage_range_valid
                                && usage_max >= usage_min
                            ) {
                                uint32_t range = usage_max - usage_min + 1;
                                if (range > global.report_count) {
                                    range = global.report_count;
                                }
                                if (range > kHidMaxButtons) {
                                    range = kHidMaxButtons;
                                }
                                if (!acc->layout.buttons_present && range > 0) {
                                    acc->layout.buttons_present = true;
                                    acc->layout.button_bit_offset = acc->bit_cursor;
                                    acc->layout.button_count = static_cast<uint8_t>(range);
                                }
                            } else if (global.usage_page == kUsagePageGenericDesktop) {
                                for (uint8_t f = 0; f < global.report_count; ++f) {
                                    uint16_t usage = 0;
                                    if (f < local_usage_count) {
                                        usage = local_usages[f];
                                    } else if (local_usage_count > 0) {
                                        // Fewer usages than fields: the last one
                                        // repeats, per HID 1.11 section 6.2.2.8.
                                        usage = local_usages[local_usage_count - 1];
                                    }
                                    HidFieldLocation *field =
                                        field_for_usage(acc->layout, usage);
                                    if (field != nullptr && !field->present) {
                                        field->present = true;
                                        field->bit_offset = static_cast<uint16_t>(
                                            acc->bit_cursor
                                            + static_cast<uint16_t>(f * global.report_size)
                                        );
                                        field->bit_size = global.report_size;
                                        field->logical_min = global.logical_min;
                                        field->logical_max = global.logical_max;
                                    }
                                }
                            }
                        }

                        acc->bit_cursor = static_cast<uint16_t>(acc->bit_cursor + field_bits);
                        acc->layout.report_bits = acc->bit_cursor;
                    }
                }

                // Every main item clears local state, collections included.
                if (tag == kMainTagInput || tag == kMainTagCollection
                    || tag == kMainTagEndCollection || type == kItemTypeMain) {
                    local_usage_count = 0;
                    usage_range_valid = false;
                    usage_min = 0;
                    usage_max = 0;
                }
                break;
            }

            default:
                break;
        }
    }

    // Pick the most gamepad-like report.
    uint8_t best_score = 0;
    for (uint8_t r = 0; r < kMaxTrackedReports; ++r) {
        if (!reports[r].used) {
            continue;
        }
        const uint8_t score = layout_score(reports[r].layout);
        if (score > best_score) {
            best_score = score;
            layout = reports[r].layout;
        }
    }

    // Sticks alone, or buttons alone, are enough to be useful. Nothing at all
    // means the descriptor was unreadable or is not a gamepad.
    layout.valid = best_score > 0;
    return layout.valid;
}
