#include <cassert>
#include <cstdint>
#include <limits>

#include "../GimbalInputGuard.hpp"

int main() {
  assert(GimbalInputGuard::IsFresh(100ULL, 50100ULL, 50000ULL));
  assert(!GimbalInputGuard::IsFresh(100ULL, 50101ULL, 50000ULL));
  assert(!GimbalInputGuard::IsFresh(100ULL, 0x100000064ULL, 50000ULL));
  assert(GimbalInputGuard::IsFresh(0xfffffffffffffff0ULL, 0x10ULL, 50000ULL));

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

  GimbalInputGuard::ModeProtocol protocol;
  assert(protocol.ObserveInputs(true) == 1U);
  const uint32_t PRE_FAULT_EPOCH = protocol.FreshEpoch();
  assert(protocol.ObserveInputs(false) == 0U);
  const uint32_t DELAYED_PRE_FAULT_SEQUENCE = 20U;
  assert(protocol.ObserveInputs(true) == 2U);
  assert(protocol.ConsumeOrdinary(DELAYED_PRE_FAULT_SEQUENCE));
  assert(!protocol.CanApplyOrdinary(DELAYED_PRE_FAULT_SEQUENCE, PRE_FAULT_EPOCH,
                                    true));

  const uint32_t POST_FRESH_SEQUENCE = 21U;
  assert(protocol.ConsumeOrdinary(POST_FRESH_SEQUENCE));
  assert(protocol.CanApplyOrdinary(POST_FRESH_SEQUENCE, protocol.FreshEpoch(),
                                   true));
  protocol.RecordOrdinaryApplied(POST_FRESH_SEQUENCE);

  const uint32_t RELAX_SEQUENCE = 30U;
  assert(protocol.ConsumeRelax(RELAX_SEQUENCE));
  assert(!protocol.ConsumeOrdinary(29U));
  assert(protocol.ConsumeOrdinary(31U));
  assert(protocol.CanApplyOrdinary(31U, protocol.FreshEpoch(), true));

  GimbalInputGuard::ModeProtocol delayed_protocol;
  assert(delayed_protocol.ObserveInputs(true) == 1U);
  assert(delayed_protocol.ConsumeOrdinary(11U));
  assert(delayed_protocol.CanApplyOrdinary(11U, 1U, true));
  delayed_protocol.RecordOrdinaryApplied(11U);
  assert(!delayed_protocol.ConsumeOrdinary(10U));
}
