#pragma once

#include <cstdint>
#include <functional>

#include "libxr_system.hpp"
#include "libxr_time.hpp"

// Deterministic replacement for the OS scheduler only. CMD, Topic, Event, PID,
// SMC and Gimbal all use their real production implementations.
namespace GimbalHost {
inline uint64_t now_us = 1000;
inline std::function<void()> thread_entry;
inline std::function<void()> after_tick;
}  // namespace GimbalHost
namespace LibXR {
class Thread {
 public:
  enum class Priority { IDLE, LOW, MEDIUM, HIGH, REALTIME, NUMBER };
  template <typename Arg>
  void Create(Arg arg, void (*entry)(Arg), const char*, size_t, Priority) {
    GimbalHost::thread_entry = [=] { entry(arg); };
  }
  static Thread Current() { return {}; }
  static uint32_t GetTime() { return GimbalHost::now_us / 1000; }
  static void Sleep(uint32_t ms) { GimbalHost::now_us += 1000 * ms; }
  static void SleepUntil(MillisecondTimestamp&, uint32_t ms) {
    GimbalHost::now_us += 1000 * ms;
    GimbalHost::after_tick();
  }
  static void Yield() {}
  operator libxr_thread_handle() { return {}; }
};
}  // namespace LibXR
