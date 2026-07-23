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
}
