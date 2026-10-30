#include "can_io_test_proto.h"

namespace ind::application::test {

uint8_t Check(const uint8_t data[8], uint8_t seed) {
    for (uint8_t i = 0; i < 7; ++i) seed ^= data[i];
    return seed;
}

Code Decode(uint32_t id, uint8_t dlc, const uint8_t data[8], Request* req) {
    if (id != rx_id || dlc != 8 || data == nullptr || req == nullptr) return Code::BadFrame;
    if (data[0] != 0xA5 || data[6] != 0 || data[7] != Check(data, 0x5A)) return Code::BadFrame;
    const uint16_t duration = static_cast<uint16_t>(data[3]) | (static_cast<uint16_t>(data[4]) << 8);
    const auto op = static_cast<Op>(data[1]);
    const uint8_t channel = data[2];
    switch (op) {
    case Op::Ping:
    case Op::Di:
    case Op::Off:
        if (channel != 0 || duration != 0) return Code::BadFrame;
        break;
    case Op::Ai:
        if (channel > 8 || duration != 0) return Code::BadFrame;
        break;
    case Op::Pulse:
        if (channel < 1 || channel > 4 || duration == 0 || duration > 200) return Code::BadFrame;
        break;
    default:
        return Code::BadFrame;
    }
    *req = {op, channel, duration, data[5]};
    return Code::Ok;
}

void Reply(Op op, uint8_t channel, uint8_t code, uint8_t seq, uint8_t low, uint8_t high, uint8_t data[8]) {
    data[0] = 0x5A;
    data[1] = 0x80U | static_cast<uint8_t>(op);
    data[2] = channel;
    data[3] = code;
    data[4] = seq;
    data[5] = low;
    data[6] = high;
    data[7] = Check(data, 0xA5);
}

} // namespace ind::application::test
