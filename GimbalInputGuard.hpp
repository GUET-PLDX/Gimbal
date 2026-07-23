#pragma once

#include <cmath>
#include <cstdint>
#include <initializer_list>

namespace GimbalInputGuard {

inline bool IsFresh(uint64_t last_us, uint64_t now_us, uint64_t timeout_us) {
  return static_cast<uint64_t>(now_us - last_us) <= timeout_us;
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

class ModeProtocol {
 public:
  uint32_t ObserveInputs(bool inputs_valid) {
    if (!inputs_valid) {
      inputs_valid_last_cycle_ = false;
      fresh_epoch_ = 0U;
      return fresh_epoch_;
    }
    if (!inputs_valid_last_cycle_) {
      ++fresh_epoch_counter_;
      if (fresh_epoch_counter_ == 0U) {
        fresh_epoch_counter_ = 1U;
      }
      fresh_epoch_ = fresh_epoch_counter_;
    }
    inputs_valid_last_cycle_ = true;
    return fresh_epoch_;
  }

  uint32_t FreshEpoch() const { return fresh_epoch_; }

  bool ConsumeOrdinary(uint32_t sequence) {
    if (!OrdinaryAfterRelax(sequence) ||
        (last_consumed_sequence_valid_ &&
         !IsSequenceAfter(sequence, last_consumed_sequence_)) ||
        (last_ordinary_sequence_valid_ &&
         !IsSequenceAfter(sequence, last_ordinary_sequence_))) {
      return false;
    }
    last_consumed_sequence_ = sequence;
    last_consumed_sequence_valid_ = true;
    return true;
  }

  bool ConsumeRelax(uint32_t sequence) {
    if (last_relax_sequence_valid_ &&
        !IsSequenceAfter(sequence, last_relax_sequence_)) {
      return false;
    }
    last_relax_sequence_ = sequence;
    last_relax_sequence_valid_ = true;
    return true;
  }

  bool OrdinaryIsCurrent(uint32_t sequence) const {
    return last_consumed_sequence_valid_ &&
           sequence == last_consumed_sequence_ &&
           OrdinaryAfterRelax(sequence) &&
           (!last_ordinary_sequence_valid_ ||
            IsSequenceAfter(sequence, last_ordinary_sequence_));
  }

  bool CanApplyOrdinary(uint32_t sequence, uint32_t request_epoch,
                        bool inputs_valid) const {
    return inputs_valid && OrdinaryIsCurrent(sequence) &&
           RequestMatchesFreshEpoch(request_epoch, fresh_epoch_);
  }

  void RecordOrdinaryApplied(uint32_t sequence) {
    last_ordinary_sequence_ = sequence;
    last_ordinary_sequence_valid_ = true;
  }

 private:
  bool OrdinaryAfterRelax(uint32_t sequence) const {
    return !last_relax_sequence_valid_ ||
           IsSequenceAfter(sequence, last_relax_sequence_);
  }

  uint32_t fresh_epoch_counter_ = 0U;
  uint32_t fresh_epoch_ = 0U;
  bool inputs_valid_last_cycle_ = false;
  uint32_t last_consumed_sequence_ = 0U;
  bool last_consumed_sequence_valid_ = false;
  uint32_t last_ordinary_sequence_ = 0U;
  bool last_ordinary_sequence_valid_ = false;
  uint32_t last_relax_sequence_ = 0U;
  bool last_relax_sequence_valid_ = false;
};

template <typename FaultLatch>
inline bool ControlAllowed(bool inputs_valid, const FaultLatch& fault_latched) {
  return inputs_valid && !static_cast<bool>(fault_latched);
}

}  // namespace GimbalInputGuard
