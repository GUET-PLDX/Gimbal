#include <Eigen/Geometry>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "GimbalKinematics.hpp"

namespace {
void Check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void CheckVector(const Eigen::Vector3f& actual, const Eigen::Vector3f& expected,
                 const char* message) {
  Check((actual - expected).norm() < 1e-5f, message);
}

// Independent construction: apply pitch to the boresight, then yaw, then base.
Eigen::Vector3f Pointing(const Eigen::Matrix3f& base, float yaw, float pitch) {
  return base * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
         Eigen::Vector3f::UnitX();
}

void TestTiltedBase() {
  const Eigen::Matrix3f base =
      (Eigen::AngleAxisf(0.9f, Eigen::Vector3f::UnitZ()) *
       Eigen::AngleAxisf(-0.4f, Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxisf(0.6f, Eigen::Vector3f::UnitX()))
          .toRotationMatrix();
  const Eigen::Matrix3f end =
      base * (Eigen::AngleAxisf(1.1f, Eigen::Vector3f::UnitZ()) *
              Eigen::AngleAxisf(-0.2f, Eigen::Vector3f::UnitY()))
                 .toRotationMatrix();
  const auto recovered = GimbalKinematics::InferBase(end, {1.1f, -0.2f});
  Check((recovered - base).norm() < 1e-5f, "tilted base recovery");
  const Eigen::Vector3f target = Pointing(base, -0.8f, 0.45f);
  const auto joint =
      GimbalKinematics::Inverse(recovered, target, {-1.0f, 1.0f}, 1.1f);
  CheckVector(Pointing(base, joint.yaw, joint.pitch), target,
              "tilted base pointing round trip");
  CheckVector(GimbalKinematics::WorldDirection({0.0f, 0.0f}),
              Eigen::Vector3f::UnitX(), "zero world target");
  CheckVector(GimbalKinematics::WorldDirection({0.0f, 0.5f}),
              {std::cos(0.5f), 0.0f, -std::sin(0.5f)},
              "right handed world pitch");
}

void TestWorkspaceAndPoles() {
  const Eigen::Matrix3f base =
      Eigen::AngleAxisf(0.7f, Eigen::Vector3f(1, 2, 3).normalized())
          .toRotationMatrix();
  for (int n = 0; n < 40; ++n) {
    const float yaw = -3.0f + 0.15f * n;
    const float pitch = -0.9f + 0.04f * n;
    CheckVector(GimbalKinematics::Forward({yaw, pitch}).col(0),
                Pointing(Eigen::Matrix3f::Identity(), yaw, pitch), "FK axes");
    const Eigen::Matrix3f end = base * GimbalKinematics::Forward({yaw, pitch});
    Check(
        (GimbalKinematics::InferBase(end, {yaw, pitch}) - base).norm() < 1e-5f,
        "fixed base under moving joints");
  }
  for (float desired_pitch : {-1.2f, -0.1f, 0.9f}) {
    const Eigen::Vector3f target = Pointing(base, 0.83f, desired_pitch);
    const auto joint =
        GimbalKinematics::Inverse(base, target, {-0.4f, 0.3f}, 0);
    const float score = Pointing(base, joint.yaw, joint.pitch).dot(target);
    for (int y = 0; y < 100; ++y) {
      for (int p = 0; p <= 70; ++p) {
        const auto reachable =
            Pointing(base, -3.14f + 6.28f * y / 100, -0.4f + 0.01f * p);
        Check(score + 2e-6f >= reachable.dot(target),
              "nearest feasible direction");
      }
    }
  }
  for (float vertical : {-1.0f, 1.0f}) {
    const auto joint = GimbalKinematics::Inverse(
        base, base * Eigen::Vector3f(0, 0, vertical), {}, 0.72f);
    Check(std::abs(joint.yaw - 0.72f) < 1e-6f, "pole retains yaw");
  }
}

void TestMappingAndGravity() {
  const GimbalKinematics::AxisMapping pitch{0.8f, -1.0f};
  Check(std::abs(pitch.Position(1.1f) + 0.3f) < 1e-6f, "pitch direction");
  Check(pitch.Velocity(2.0f) == -2.0f, "CAN velocity direction");
  Check(pitch.MotorTorque(-0.6f) == 0.6f, "motor torque direction");
  Check(pitch.JointTorque(0.6f) == -0.6f, "applied torque direction");
  const auto limits = pitch.Limits(0.5f, 1.8f);
  Check(std::abs(limits.min + 1.0f) < 1e-6f &&
            std::abs(limits.max - 0.3f) < 1e-6f,
        "mapped mechanical limits");
  const GimbalKinematics::AxisMapping yaw{4.8f, 1.0f};
  Check(std::abs(yaw.Position(0.1f) - 1.5831853f) < 1e-6f, "encoder wrap");
  for (float direction : {-1.0f, 1.0f}) {
    for (float app_pitch : {-0.7f, 0.0f, 0.5f}) {
      const Eigen::Matrix3f end =
          (Eigen::AngleAxisf(0.8f, Eigen::Vector3f::UnitZ()) *
           Eigen::AngleAxisf(-app_pitch, Eigen::Vector3f::UnitY()))
              .toRotationMatrix();
      const float torque =
          GimbalKinematics::GravityTorque(end, 0.6f, -0.1f, direction);
      Check(std::abs(direction * torque + 0.6f * std::sin(app_pitch - 0.1f)) <
                1e-6f,
            "level gravity agrees with existing motor model");
    }
  }
  const Eigen::Matrix3f end =
      Eigen::AngleAxisf(1.2f, Eigen::Vector3f(1, 2, 3).normalized())
          .toRotationMatrix();
  const Eigen::Vector3f weighted_arm(-0.6f * std::sin(-0.1f), 0,
                                     0.6f * std::cos(-0.1f));
  constexpr float H = 0.001f;
  const Eigen::Vector3f plus =
      end * Eigen::AngleAxisf(H, Eigen::Vector3f::UnitY()) * weighted_arm;
  const Eigen::Vector3f minus =
      end * Eigen::AngleAxisf(-H, Eigen::Vector3f::UnitY()) * weighted_arm;
  Check(std::abs(GimbalKinematics::GravityTorque(end, 0.6f, -0.1f, -1) -
                 (plus.z() - minus.z()) / (2 * H)) < 5e-5f,
        "tilted gravity equals potential energy gradient");
}

void TestReferenceDerivatives() {
  GimbalKinematics::ReferenceTracker tracker(30.0f);
  auto ref = tracker.Update({3.13f, 0.1f}, 0.001f);
  Check(ref.velocity.yaw == 0 && ref.acceleration.pitch == 0,
        "first reference has zero derivatives");
  ref = tracker.Update({-3.13f, 0.102f}, 0.001f);
  Check(ref.position.yaw > 3.14f && ref.velocity.yaw > 0 &&
            ref.velocity.yaw < 24.0f,
        "yaw wrap has no 2pi derivative spike");
  for (int n = 0; n < 400; ++n) ref = tracker.Update({-3.13f, 0.102f}, 0.001f);
  Check(std::abs(ref.velocity.yaw) < 1e-5f &&
            std::abs(ref.acceleration.pitch) < 1e-5f,
        "static derivatives settle");
  tracker.Reset();
  ref = tracker.Update({-1.0f, -0.2f}, 0.001f);
  Check(ref.velocity.yaw == 0 && ref.acceleration.pitch == 0,
        "reentry does not difference across control spaces");
  for (int n = 1; n <= 200; ++n) {
    // Fixed world X target; a base turning at +0.4 rad/s requires -0.4 yaw.
    const Eigen::Matrix3f base =
        Eigen::AngleAxisf(0.4f * n * 0.001f, Eigen::Vector3f::UnitZ())
            .toRotationMatrix();
    const auto joint =
        GimbalKinematics::Inverse(base, Eigen::Vector3f::UnitX(), {}, 0);
    ref = tracker.Update(joint, 0.001f);
  }
  Check(std::abs(ref.velocity.yaw + 0.4f) < 0.001f,
        "moving base contributes joint reference velocity");
  Check(std::abs(ref.acceleration.yaw) < 0.01f,
        "constant speed reference acceleration settles");
  Check(std::abs(GimbalKinematics::ShortestYawDelta(GimbalKinematics::PI, 0) +
                 GimbalKinematics::PI) < 1e-6f,
        "half turn matches SMC cycle convention");
}

void TestSampleHoldAndVariableCycle() {
  GimbalKinematics::ReferenceTracker tracker(30.0f);
  tracker.Update({}, 0.001f);
  // A 2 ms command hold: one duplicate reference has zero raw derivative;
  // at the next sample the same displacement spans the current actual dt.
  auto ref = tracker.Update({}, 0.001f);
  Check(ref.velocity.yaw == 0 && ref.acceleration.yaw == 0,
        "duplicate initial reference stays still");
  ref = tracker.Update({0.002f, -0.002f}, 0.002f);
  const float a = 1 - std::exp(-2 * GimbalKinematics::PI * 30 * 0.002f);
  Check(std::abs(ref.velocity.yaw - a) < 1e-6f,
        "variable cycle uses seconds for reference speed");
  Check(std::abs(ref.acceleration.pitch + a * a / 0.002f) < 2e-4f,
        "variable cycle reference acceleration");
  ref = tracker.Update({0.002f, -0.002f}, 0.001f);
  const float decay = std::exp(-2 * GimbalKinematics::PI * 30 * 0.001f);
  Check(std::abs(ref.velocity.yaw - a * decay) < 1e-6f,
        "held sample decays through the reference filter");
  const float previous_speed = ref.velocity.yaw;
  const auto paused = tracker.Update({1.0f, 1.0f}, 0.0f);
  Check(paused.velocity.yaw == previous_speed,
        "zero dt does not corrupt derivative history");
}
}  // namespace

int main() {
  TestTiltedBase();
  TestWorkspaceAndPoles();
  TestMappingAndGravity();
  TestReferenceDerivatives();
  TestSampleHoldAndVariableCycle();
}
