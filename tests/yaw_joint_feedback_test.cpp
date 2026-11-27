#undef NDEBUG
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "YawJointFeedback.hpp"
#include "libxr.hpp"

using namespace Pldx;

static Motor::Feedback Frame(float abs_angle, float omega, uint64_t us, uint16_t seq) {
  Motor::Feedback feedback{};
  feedback.abs_angle = abs_angle;
  feedback.omega = omega;
  feedback.received_time_us = us;
  feedback.sequence = seq;
  return feedback;
}

int main() {
  uint16_t last = 0U;
  YawJointFeedback out{};
  uint64_t us = 0U;

  // Angle and velocity come from one motor frame; zero is subtracted once.
  assert(MakeYawJointFeedback(Frame(2.0F, -0.25F, 0x060504030201ULL, 1U), true,
                              0.5F, last, out, us));
  assert(out.angle_rad == 1.5F && out.velocity_rad_s == -0.25F);
  assert(us == 0x060504030201ULL && last == 1U);

  // The same frame is never published twice.
  assert(!MakeYawJointFeedback(Frame(2.0F, -0.25F, 0x060504030201ULL, 1U), true,
                               0.5F, last, out, us));

  // Offline, error and non-finite frames are not samples.
  assert(!MakeYawJointFeedback(Frame(2.0F, 0.0F, 10U, 2U), false, 0.0F, last, out, us));
  auto bad = Frame(2.0F, 0.0F, 10U, 2U);
  bad.error_id = 1U;
  assert(!MakeYawJointFeedback(bad, true, 0.0F, last, out, us));
  assert(!MakeYawJointFeedback(
      Frame(2.0F, std::numeric_limits<float>::quiet_NaN(), 10U, 2U), true, 0.0F, last, out, us));
  assert(!MakeYawJointFeedback(Frame(2.0F, 0.0F, 0U, 2U), true, 0.0F, last, out, us));

  // 48-bit packet counter boundary: the source time wraps like the header does.
  assert(MakeYawJointFeedback(Frame(0.0F, 0.0F, (1ULL << 48U) + 5U, 3U), true, 0.0F,
                              last, out, us));
  assert(us == 5U);

  // Uint16 sequence rollover still publishes the next frame.
  last = 65535U;
  assert(MakeYawJointFeedback(Frame(0.0F, 0.0F, 7U, 0U), true, 0.0F, last, out, us));

  // Payload bytes sent over the USB link (the sample time is in the packet header).
  const YawJointFeedback sent{1.5F, -0.25F};
  std::array<uint8_t, sizeof(sent)> bytes{};
  std::memcpy(bytes.data(), &sent, bytes.size());
  const std::array<uint8_t, 8> payload{0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x80, 0xBE};
  assert(bytes == payload);
  return 0;
}
