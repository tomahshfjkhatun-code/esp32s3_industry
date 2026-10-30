#pragma once

#include <stdint.h>

namespace ind::application::test {

inline constexpr uint32_t rx_id = 0x18FF7020U;
inline constexpr uint32_t tx_id = 0x18FF7021U;

enum class Op : uint8_t { Ping = 1, Di = 2, Ai = 3, Pulse = 4, Off = 5 };
enum class Code : uint8_t {
    Ok = 0,
    BadFrame = 1,
    NoNetwork = 2,
    Polarity = 3,
    Unsafe = 4,
    Busy = 5,
    BadCan = 6,
    Stale = 7,
    Timing = 8
};

struct Request {
    Op op = Op::Ping;
    uint8_t channel = 0;
    uint16_t duration = 0;
    uint8_t seq = 0;
};

uint8_t Check(const uint8_t data[8], uint8_t seed);
Code Decode(uint32_t id, uint8_t dlc, const uint8_t data[8], Request* req);
void Reply(Op op, uint8_t channel, uint8_t code, uint8_t seq, uint8_t low, uint8_t high, uint8_t data[8]);

} // namespace ind::application::test
