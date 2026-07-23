#include <cassert>
#include <cmath>

#include "../PatrolTrajectory.hpp"

int main() {
  constexpr float CENTER = 0.20f;
  constexpr float AMPLITUDE = 0.455f;

  for (int i = 0; i <= 600000; ++i) {
    const float TARGET = PatrolTrajectory::PitchTarget(
        CENTER, AMPLITUDE, 10.0f, static_cast<float>(i) * 0.002f);
    assert(std::isfinite(TARGET));
    assert(TARGET >= CENTER - AMPLITUDE - 1e-5f);
    assert(TARGET <= CENTER + AMPLITUDE + 1e-5f);
  }
}
