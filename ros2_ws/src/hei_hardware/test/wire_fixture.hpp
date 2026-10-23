#pragma once
// Independent expected wire payloads for the PTY peer; test-only.
#include <array>
#include <cstdint>

namespace hei_hardware::damiao
{
using Payload = std::array<uint8_t, 8>;
struct Frame { uint32_t id; Payload data; };
Frame position_velocity(uint16_t motor_id, double position, double speed);
}  // namespace hei_hardware::damiao
