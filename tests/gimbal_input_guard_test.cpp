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
  bool inputs_fresh_observed = false;
  const bool STALE_REQUEST_REARM_ELIGIBLE = inputs_fresh_observed;
  GimbalInputGuard::UpdateFaultLatch(false, input_fault_latched);
  assert(input_fault_latched);
  if (GimbalInputGuard::AcceptActiveRequest(false, input_fault_latched)) {
    active_mode = true;
  }
  assert(!active_mode);
  assert(input_fault_latched);

  GimbalInputGuard::UpdateFaultLatch(true, input_fault_latched);
  assert(!GimbalInputGuard::ActiveRequestCanRearm(STALE_REQUEST_REARM_ELIGIBLE,
                                                  true));
  assert(input_fault_latched);
  assert(!GimbalInputGuard::ControlAllowed(true, input_fault_latched));
  assert(!active_mode);

  inputs_fresh_observed = true;
  const bool POST_FRESH_REQUEST_REARM_ELIGIBLE = inputs_fresh_observed;
  if (GimbalInputGuard::ActiveRequestCanRearm(POST_FRESH_REQUEST_REARM_ELIGIBLE,
                                              true) &&
      GimbalInputGuard::AcceptActiveRequest(true, input_fault_latched)) {
    active_mode = true;
  }
  assert(active_mode);
  assert(!input_fault_latched);
  assert(GimbalInputGuard::ControlAllowed(true, input_fault_latched));
}
