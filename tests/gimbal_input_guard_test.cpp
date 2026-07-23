#include <cassert>
#include <cstdint>
#include <limits>

#include "../GimbalInputGuard.hpp"

int main() {
  assert(GimbalInputGuard::IsFresh(100U, 50100U, 50000U));
  assert(!GimbalInputGuard::IsFresh(100U, 50101U, 50000U));
  assert(GimbalInputGuard::IsFresh(0xfffffff0U, 0x10U, 50000U));
  assert(GimbalInputGuard::IsFresh(0U, 0U, 50000U));

  assert(GimbalInputGuard::AllFinite({0.0f, 1.0f, -1.0f}));
  assert(!GimbalInputGuard::AllFinite(
      {0.0f, std::numeric_limits<float>::quiet_NaN()}));
  assert(!GimbalInputGuard::AllFinite(
      {std::numeric_limits<float>::infinity(), 0.0f}));

  bool input_fault_latched = false;
  bool active_mode = false;
  const uint32_t STALE_REQUEST_EPOCH = 3U;
  GimbalInputGuard::UpdateFaultLatch(false, input_fault_latched);
  assert(input_fault_latched);
  if (GimbalInputGuard::AcceptActiveRequest(false, input_fault_latched)) {
    active_mode = true;
  }
  assert(!active_mode);
  assert(input_fault_latched);

  GimbalInputGuard::UpdateFaultLatch(true, input_fault_latched);
  const uint32_t RECOVERED_FRESH_EPOCH = 4U;
  assert(!GimbalInputGuard::RequestMatchesFreshEpoch(STALE_REQUEST_EPOCH,
                                                     RECOVERED_FRESH_EPOCH));
  assert(input_fault_latched);
  assert(!GimbalInputGuard::ControlAllowed(true, input_fault_latched));
  assert(!active_mode);

  const uint32_t POST_FRESH_REQUEST_EPOCH = RECOVERED_FRESH_EPOCH;
  if (GimbalInputGuard::RequestMatchesFreshEpoch(POST_FRESH_REQUEST_EPOCH,
                                                 RECOVERED_FRESH_EPOCH) &&
      GimbalInputGuard::AcceptActiveRequest(true, input_fault_latched)) {
    active_mode = true;
  }
  assert(active_mode);
  assert(!input_fault_latched);
  assert(GimbalInputGuard::ControlAllowed(true, input_fault_latched));

  assert(!GimbalInputGuard::RequestMatchesFreshEpoch(3U, 4U));
  assert(GimbalInputGuard::RequestMatchesFreshEpoch(4U, 4U));
  assert(!GimbalInputGuard::RequestMatchesFreshEpoch(0U, 0U));

  const uint32_t DELAYED_ACTIVE_SEQUENCE = 10U;
  const uint32_t RELAX_CUTOFF_SEQUENCE = 11U;
  const uint32_t POST_RELAX_ACTIVE_SEQUENCE = 12U;
  assert(!GimbalInputGuard::IsSequenceAfter(DELAYED_ACTIVE_SEQUENCE,
                                            RELAX_CUTOFF_SEQUENCE));
  assert(!GimbalInputGuard::IsSequenceAfter(RELAX_CUTOFF_SEQUENCE,
                                            RELAX_CUTOFF_SEQUENCE));
  assert(GimbalInputGuard::IsSequenceAfter(POST_RELAX_ACTIVE_SEQUENCE,
                                           RELAX_CUTOFF_SEQUENCE));
  assert(GimbalInputGuard::IsSequenceAfter(1U, 0xffffffffU));
  assert(!GimbalInputGuard::IsSequenceAfter(0xfffffffeU, 1U));
  assert(!GimbalInputGuard::IsSequenceAfter(0x80000001U, 1U));
}
