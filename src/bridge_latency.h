#ifndef DS5_BRIDGE_BRIDGE_LATENCY_H
#define DS5_BRIDGE_BRIDGE_LATENCY_H

#include <cstdint>

//
// Timing for the two legs of the chain the bridge can actually observe:
//
//   pad --Bluetooth--> [ bridge ] --USB--> Windows
//        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//        interval               bridge add
//
// "interval" is the gap between consecutive input reports from the pad, which
// is the dominant term and is set by the pad's Bluetooth reporting rate.
// "bridge add" is report-received to report-queued-to-USB, i.e. what this
// firmware itself costs.
//
// Deliberately header-only with inline state: the update calls sit inside
// on_bt_data and interrupt_loop, which are relocated into SRAM. Inlining keeps
// them from becoming calls into flash-resident code.
//
// Not measured, and not measurable from here: the pad's own scan-to-radio
// delay, and everything past the USB endpoint on the Windows side.
//

struct BridgeLatencyStats {
    uint32_t reports;
    uint32_t interval_min_us;
    uint32_t interval_max_us;
    uint64_t interval_sum_us;
    uint32_t interval_samples;
    uint32_t idle_gaps;

    uint32_t sends;
    uint32_t add_min_us;
    uint32_t add_max_us;
    uint64_t add_sum_us;
};

// A gap longer than this is the pad going quiet, not a late report. Counted
// separately so one idle stretch does not swallow the max. Any real Bluetooth
// gamepad interval is well under 20 ms, so 100 ms is generous; the original
// 500 ms let idle gaps through and made MAX meaningless.
inline constexpr uint32_t kBridgeLatencyIdleGapUs = 100000;

// Reports discarded after a connection before timing starts. USB is still
// attaching over the first moments and input is deliberately held quiet, so
// early samples measure the startup sequence rather than steady-state latency.
inline constexpr uint32_t kBridgeLatencyWarmupReports = 100;

inline BridgeLatencyStats g_bridge_latency{};
inline uint32_t g_bridge_last_report_us = 0;
inline bool g_bridge_have_last_report = false;
inline uint32_t g_bridge_pending_report_us = 0;

inline void bridge_latency_reset() {
    g_bridge_latency = BridgeLatencyStats{};
    g_bridge_latency.interval_min_us = UINT32_MAX;
    g_bridge_latency.add_min_us = UINT32_MAX;
    g_bridge_have_last_report = false;
    g_bridge_pending_report_us = 0;
}

// Called from on_bt_data for every accepted input report.
inline void bridge_latency_note_report(uint32_t now_us) {
    g_bridge_latency.reports++;
    if (g_bridge_latency.reports <= kBridgeLatencyWarmupReports) {
        g_bridge_last_report_us = now_us;
        g_bridge_have_last_report = true;
        g_bridge_pending_report_us = 0;
        return;
    }
    if (g_bridge_have_last_report) {
        const uint32_t delta = now_us - g_bridge_last_report_us;
        if (delta >= kBridgeLatencyIdleGapUs) {
            g_bridge_latency.idle_gaps++;
        } else {
            if (delta < g_bridge_latency.interval_min_us) {
                g_bridge_latency.interval_min_us = delta;
            }
            if (delta > g_bridge_latency.interval_max_us) {
                g_bridge_latency.interval_max_us = delta;
            }
            g_bridge_latency.interval_sum_us += delta;
            g_bridge_latency.interval_samples++;
        }
    }
    g_bridge_last_report_us = now_us;
    g_bridge_have_last_report = true;
    g_bridge_pending_report_us = now_us;
}

// Called from interrupt_loop once a report has been handed to USB.
inline void bridge_latency_note_sent(uint32_t now_us) {
    if (g_bridge_pending_report_us == 0) {
        return;
    }
    const uint32_t delta = now_us - g_bridge_pending_report_us;
    g_bridge_pending_report_us = 0;
    if (delta >= kBridgeLatencyIdleGapUs) {
        return;
    }
    g_bridge_latency.sends++;
    if (delta < g_bridge_latency.add_min_us) {
        g_bridge_latency.add_min_us = delta;
    }
    if (delta > g_bridge_latency.add_max_us) {
        g_bridge_latency.add_max_us = delta;
    }
    g_bridge_latency.add_sum_us += delta;
}

inline BridgeLatencyStats bridge_latency_snapshot() {
    return g_bridge_latency;
}

#endif // DS5_BRIDGE_BRIDGE_LATENCY_H
