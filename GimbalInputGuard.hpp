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

template <typename FaultLatch>
inline void UpdateFaultLatch(bool inputs_valid, FaultLatch& fault_latched) {
  if (!inputs_valid) {
    fault_latched = true;
  }
}

template <typename FaultLatch>
inline bool AcceptActiveRequest(bool inputs_valid, FaultLatch& fault_latched) {
  if (!inputs_valid) {
    fault_latched = true;
    return false;
  }
  fault_latched = false;
  return true;
}

inline bool RequestMatchesFreshEpoch(uint32_t request_epoch,
                                     uint32_t current_epoch) {
  return request_epoch != 0U && request_epoch == current_epoch;
}

inline bool IsSequenceAfter(uint32_t sequence, uint32_t reference) {
  // Compared requests must be less than half the uint32_t sequence space apart.
  const uint32_t DELTA = sequence - reference;
  return DELTA != 0U && DELTA < 0x80000000U;
}

template <typename FaultLatch>
inline bool ControlAllowed(bool inputs_valid, const FaultLatch& fault_latched) {
  return inputs_valid && !static_cast<bool>(fault_latched);
}

}  // namespace GimbalInputGuard
