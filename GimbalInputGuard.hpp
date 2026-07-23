#pragma once

#include <cmath>
#include <cstdint>
#include <initializer_list>

namespace GimbalInputGuard {

inline bool IsFresh(uint32_t last_us, uint32_t now_us, uint32_t timeout_us) {
  return static_cast<uint32_t>(now_us - last_us) <= timeout_us;
}

inline bool AllFinite(std::initializer_list<float> values) {
  for (const float VALUE : values) {
    if (!std::isfinite(VALUE)) {
      return false;
    }
  }
  return true;
}

}  // namespace GimbalInputGuard
