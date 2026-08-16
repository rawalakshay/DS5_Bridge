#ifndef DS5_BRIDGE_SWITCH_RUMBLE_H
#define DS5_BRIDGE_SWITCH_RUMBLE_H

#include <cstdint>

//
// Switch-style rumble encoding, used for generic pads that accept output
// report 0x10.
//
// Each motor is driven as two bands. The high-frequency band carries its
// amplitude in the second byte alongside the top bits of the frequency; the
// low-frequency band carries frequency and amplitude in the last two. The
// amplitude scales are encoded ranges, not linear PWM: writing a raw 0-255 into
// them produces silence rather than a loud buzz, which is why intensity has to
// go through here rather than straight onto the wire.
//
// Header-only and free of I/O so tests/firmware can exercise it directly.
//

inline constexpr uint8_t kSwitchRumbleMotorBytes = 4;

// Frequency is held at the neutral frame's value so amplitude is the only thing
// that varies. hf 0x0100 is what makes byte 1 read 0x01 at rest.
inline constexpr uint16_t kSwitchRumbleHighFrequency = 0x0100;
inline constexpr uint8_t kSwitchRumbleLowFrequency = 0x40;
inline constexpr uint8_t kSwitchRumbleHighAmplitudeMax = 0xC8; // 0x00..0xC8, stays even.
inline constexpr uint8_t kSwitchRumbleLowAmplitudeZero = 0x40; // 0x40..0x72.
inline constexpr uint8_t kSwitchRumbleLowAmplitudeSpan = 0x32;

// Writes kSwitchRumbleMotorBytes for one motor. Intensity 0 reproduces the
// neutral bytes exactly -- that specific frame is the one verified to stop the
// motors, so the zero case must not drift from it.
inline void switch_rumble_encode_motor(uint8_t *out, uint8_t intensity) {
    const uint8_t high_amplitude = static_cast<uint8_t>(
        ((static_cast<uint32_t>(intensity) * kSwitchRumbleHighAmplitudeMax) / 255u) & 0xFEu
    );
    const uint8_t low_amplitude = static_cast<uint8_t>(
        kSwitchRumbleLowAmplitudeZero
        + ((static_cast<uint32_t>(intensity) * kSwitchRumbleLowAmplitudeSpan) / 255u)
    );

    out[0] = static_cast<uint8_t>(kSwitchRumbleHighFrequency & 0xFF);
    out[1] = static_cast<uint8_t>(
        high_amplitude + ((kSwitchRumbleHighFrequency >> 8) & 0xFF)
    );
    out[2] = kSwitchRumbleLowFrequency;
    out[3] = low_amplitude;
}

// Builds the full 10-byte output report: 0x10, a 4-bit sequence counter, then
// the left and right motors.
inline void switch_rumble_encode_frame(
    uint8_t *out,
    uint8_t sequence,
    uint8_t left,
    uint8_t right
) {
    out[0] = 0x10;
    out[1] = static_cast<uint8_t>(sequence & 0x0F);
    switch_rumble_encode_motor(out + 2, left);
    switch_rumble_encode_motor(out + 2 + kSwitchRumbleMotorBytes, right);
}

inline constexpr uint8_t kSwitchRumbleFrameBytes = 2 + (2 * kSwitchRumbleMotorBytes);

#endif // DS5_BRIDGE_SWITCH_RUMBLE_H
