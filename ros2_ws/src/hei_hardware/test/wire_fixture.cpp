#include "wire_fixture.hpp"
#include <cstring>

namespace hei_hardware::damiao
{
Frame position_velocity(uint16_t motor_id, double position, double speed)
{
  Frame result{static_cast<uint32_t>(0x100 + motor_id), {}};
  for (unsigned offset : {0u, 4u}) {
    const float value = offset == 0 ? position : speed;
    uint32_t bits;
    static_assert(sizeof(value) == sizeof(bits));
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned i = 0; i < 4; ++i) { result.data[offset + i] = (bits >> (8 * i)) & 0xff; }
  }
  return result;
}
}  // namespace hei_hardware::damiao
