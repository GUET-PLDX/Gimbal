#include <cmath>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>

#include "Gimbal.hpp"

extern "C" void libxr_fatal_error(const char* file, uint32_t line, bool) {
  std::cerr << "LibXR fatal: " << file << ':' << line << '\n';
  std::abort();
}

namespace LibXR {
MicrosecondTimestamp Timebase::GetMicroseconds() { return GimbalHost::now_us; }
MillisecondTimestamp Timebase::GetMilliseconds() {
  return static_cast<uint32_t>(GimbalHost::now_us / 1000);
}
}  // namespace LibXR

namespace {
void Near(float actual, float expected, const char* message,
          float tolerance = 2e-4f) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
    std::cerr << message << ": actual=" << actual << " expected=" << expected
              << '\n';
    std::exit(1);
  }
}
class FakeMotor : public Motor {
 public:
  Feedback feedback{};
  MotorCmd command{};
  int relaxed = 0;
  void Enable() override { feedback.state = 1; }
  void Disable() override { command.torque = 0; }
  void Relax() override {
    command.torque = 0;
    ++relaxed;
  }
  LibXR::ErrorCode Update() override { return LibXR::ErrorCode::OK; }
  bool IsOnline() const override { return true; }
  const Feedback& GetFeedback() override { return feedback; }
  void Control(const MotorCmd& cmd) override { command = cmd; }
  void ClearError() override {}
  void SaveZeroPoint() override {}
};

struct Complete {};
struct Step {
  std::function<void()> setup;
  std::function<void()> check;
};

Gimbal::Param Parameters() {
  Gimbal::Param p{};
  p.task_stack_depth = 2048;
  p.manual.pid_pit_angle = {.k = 1,
                            .p = 2,
                            .i = 0,
                            .d = 0.1f,
                            .i_limit = 0,
                            .out_limit = 0,
                            .cycle = true};
  p.manual.pid_pit_omega = {.k = 1,
                            .p = 0.5f,
                            .i = 0,
                            .d = 0,
                            .i_limit = 0,
                            .out_limit = 0,
                            .cycle = false};
  p.manual.yaw_smc = {.c = 1,
                      .k = 2,
                      .epsilon = 0,
                      .q = 21,
                      .p = 27,
                      .error_deadband_rad = 0,
                      .ftsmc_switch_rad = 0.25f,
                      .sat_boundary = 1,
                      .ftsmc_enable = false};
  p.auto_aim = p.manual;
  p.yaw_output_limits = {.torque_soft_limit_nm = 0,
                         .torque_min_nm = -1000,
                         .torque_max_nm = 1000,
                         .torque_slew_rate_nm_s = 1000,
                         .torque_slew_enable = false};
  p.pit_min_angle = 0.5f;
  p.pit_max_angle = 1.8f;
  p.pit_zero = 0.8f;
  p.yaw_zero = 4.78755426f;
  p.kinematics = {
      .yaw_direction = -1, .pitch_direction = -1, .reference_cutoff_hz = 30};
  p.reverse_flag = true;
  p.j_pit = 0.014f;
  p.j_yaw = 0.04f;
  p.pit_lc = 0.6f;
  p.pit_theta = -0.1f;
  p.euler_topic_name = "gimbal_euler";
  p.gyro_topic_name = "gimbal_gyro";
  p.quaternion_topic_name = "ahrs_quaternion";
  p.gimbal_cmd_topic_name = "gimbal_cmd";
  return p;
}

void TestRealControlLoop() {
  auto euler_topic =
      LibXR::Topic::CreateTopic<LibXR::EulerAngle<float>>("gimbal_euler");
  auto gyro_topic = LibXR::Topic::CreateTopic<Eigen::Vector3f>("gimbal_gyro");
  auto quaternion_topic =
      LibXR::Topic::CreateTopic<LibXR::Quaternion<float>>("ahrs_quaternion");
  CMD cmd(CMD::Mode::CMD_AUTO_CTRL);
  FakeMotor pitch, yaw;
  const auto param = Parameters();
  pitch.feedback.state = yaw.feedback.state = 1;
  pitch.feedback.abs_angle = param.pit_zero;
  yaw.feedback.abs_angle = param.yaw_zero;
  Gimbal gimbal(cmd, pitch, yaw, param);
  LibXR::Topic::ASyncSubscriber<float> published_yaw_rate("yawmotor_omega");
  LibXR::Topic::ASyncSubscriber<float> published_pitch_angle(
      "pitchmotor_angle");
  LibXR::Topic::ASyncSubscriber<float> published_pitch_rate("pitchmotor_omega");
  std::deque<Step> steps;
  std::function<void()> check_previous = [] {};
  auto enqueue = [&](std::function<void()> setup, std::function<void()> check) {
    steps.push_back({setup, check});
  };
  LibXR::EulerAngle<float> euler(0, 0, 0);
  Eigen::Vector3f gyro(0, 7, 9);  // Deliberately differs from encoder omega.
  LibXR::Quaternion<float> attitude;
  CMD::Data ai{};
  ai.gimbal_online = true;
  ai.gimbal.yaw = 0.2f;
  ai.gimbal.pit = 0.1f;
  ai.gimbal.yaw_dot = 5;  // New path must derive these from IK instead.
  ai.gimbal.pit_dot = 6;
  CMD::Data rc{};
  rc.chassis_online = rc.gimbal_online = true;
  cmd.FeedRC(rc);
  auto publish = [&] {
    euler_topic.Publish(euler);
    gyro_topic.Publish(gyro);
    quaternion_topic.Publish(attitude);
    cmd.FeedAIGimbal(ai);
  };
  enqueue(
      [&] {
        gimbal.GetEvent().Active(
            static_cast<uint32_t>(GimbalEvent::SET_VISION_AUTO_AIM));
        pitch.feedback.omega = 0.4f;
        yaw.feedback.omega = 0.3f;
        publish();
      },
      [] {
      });  // First tick selects the new coordinate space and latches posture.
  for (int i = 0; i < 500; ++i) enqueue(publish, [] {});
  enqueue(
      [&] {
        published_yaw_rate.StartWaiting();
        published_pitch_rate.StartWaiting();
        published_pitch_angle.StartWaiting();
        publish();
      },
      [&] {
        // Joint pitch=-0.1, feedback=0, encoder velocity=-0.4.
        // Angle PID gives -0.2 + 0.04=-0.16; velocity PID gives 0.12.
        // Gravity is joint -0.6*sin(0.1); torque maps back to the motor with
        // -1.
        Near(pitch.command.torque, -0.12f + 0.6f * std::sin(0.1f),
             "ordinary aim Pitch uses encoder angle/rate");
        // Joint yaw=+0.2, q=0, qdot=-0.3: e=-0.2, edot=-0.3, s=-0.5.
        Near(yaw.command.torque, -0.04f * (0.3f + 1.0f),
             "ordinary aim Yaw uses encoder rate and torque direction");
        Near(published_yaw_rate.GetData(), 9, "public yaw speed remains IMU");
        Near(published_pitch_rate.GetData(), -7,
             "public pitch speed remains IMU");
        Near(published_pitch_angle.GetData(), 0,
             "public pitch angle remains encoder relative");
      });
  enqueue(
      [&] {
        ai.gimbal.pit = 0.2f;
        publish();
      },
      [&] {
        // Worked one-sample reference step (-0.1 -> -0.2 rad). The filtered
        // reference speed is -100*a, the total speed changes by -0.2-100*a.
        const float a = 1 - std::exp(-2 * GimbalKinematics::PI * 30 * 0.001f);
        const float expected_joint = 0.5f * (0.04f - 100 * a) +
                                     14 * (-0.2f - 100 * a) -
                                     0.6f * std::sin(0.1f);
        Near(pitch.command.torque, -expected_joint,
             "total Pitch velocity acceleration is fed forward exactly once",
             0.002f);
      });
  enqueue(
      [&] {
        ai.gimbal.pit = 0.1f;
        publish();
      },
      [] {});
  for (int i = 0; i < 300; ++i) enqueue(publish, [] {});
  // Aim -> buff keeps the auto_aim profile but must switch coordinates/reset.
  enqueue(
      [&] {
        gimbal.GetEvent().Active(
            static_cast<uint32_t>(GimbalEvent::SET_VISION_SMALL_BUFF));
        publish();
      },
      [] {});
  enqueue(publish, [&] {
    Near(pitch.command.torque, 6.95f + 0.6f * std::sin(0.1f),
         "small buff retains world commands and IMU feedback");
    Near(yaw.command.torque, -0.464f,
         "small buff retains world Yaw derivatives");
  });
  enqueue(
      [&] {
        gimbal.GetEvent().Active(
            static_cast<uint32_t>(GimbalEvent::SET_VISION_BIG_BUFF));
        publish();
      },
      [] {});
  enqueue(publish, [&] {
    Near(pitch.command.torque, 6.95f + 0.6f * std::sin(0.1f),
         "big buff retains original Pitch controller");
  });
  enqueue(
      [&] {
        gimbal.GetEvent().Active(
            static_cast<uint32_t>(GimbalEvent::SET_VISION_AUTO_AIM));
        publish();
      },
      [&] {
        Near(pitch.command.torque, -0.12f + 0.6f * std::sin(0.1f),
             "buff to aim seeds reference derivatives and total speed history");
        Near(yaw.command.torque, -0.052f,
             "buff to aim resets reference derivatives");
      });
  // Encoder joint displacement: a level base, qyaw=0.15, qpitch=-0.2.
  enqueue(
      [&] {
        yaw.feedback.abs_angle = param.yaw_zero - 0.15f;
        pitch.feedback.abs_angle = param.pit_zero + 0.2f;
        attitude = Eigen::Quaternionf(
            Eigen::AngleAxisf(0.15f, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(-0.2f, Eigen::Vector3f::UnitY()));
        euler = attitude.ToEulerAngleZYX();
        publish();
      },
      [] {});
  for (int i = 0; i < 300; ++i) enqueue(publish, [] {});
  enqueue(publish, [&] {
    Near(pitch.command.torque, -0.32f - 0.6f * std::sin(0.1f),
         "nonzero encoder position closes Pitch joint loop");
    Near(yaw.command.torque, -0.04f * (0.3f + 0.7f),
         "nonzero encoder position closes Yaw joint loop");
  });
  // Command past the lower mechanical joint limit. New path clips joint pitch
  // to -1.0, regardless of the old world-Pitch bound calculation.
  enqueue(
      [&] {
        ai.gimbal.pit = 1.3f;
        publish();
      },
      [] {});
  for (int i = 0; i < 300; ++i) enqueue(publish, [] {});
  enqueue(publish, [&] {
    Near(pitch.command.torque, 0.58f - 0.6f * std::sin(0.1f),
         "Pitch mechanical encoder limit is applied in joint coordinates");
  });
  enqueue(
      [&] {
        const Eigen::Matrix3f base =
            Eigen::AngleAxisf(0.8f, Eigen::Vector3f::UnitX())
                .toRotationMatrix();
        const Eigen::Matrix3f end =
            base * (Eigen::AngleAxisf(0.15f, Eigen::Vector3f::UnitZ()) *
                    Eigen::AngleAxisf(-0.2f, Eigen::Vector3f::UnitY()))
                       .toRotationMatrix();
        attitude = Eigen::Quaternionf(end);
        euler = attitude.ToEulerAngleZYX();
        const Eigen::Vector3f target =
            base * Eigen::AngleAxisf(-0.3f, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitY()) *
            Eigen::Vector3f::UnitX();
        ai.gimbal.yaw = std::atan2(target.y(), target.x());
        ai.gimbal.pit =
            std::atan2(target.z(), std::hypot(target.x(), target.y()));
        pitch.feedback.omega = yaw.feedback.omega = 0;
        publish();
      },
      [] {});
  for (int i = 0; i < 300; ++i) enqueue(publish, [] {});
  enqueue(publish, [&] {
    const Eigen::Vector3f arm(-0.6f * std::sin(-0.1f), 0,
                              0.6f * std::cos(-0.1f));
    const Eigen::Matrix3f end = attitude.ToRotationMatrix();
    const Eigen::Vector3f plus =
        end * Eigen::AngleAxisf(0.001f, Eigen::Vector3f::UnitY()) * arm;
    const Eigen::Vector3f minus =
        end * Eigen::AngleAxisf(-0.001f, Eigen::Vector3f::UnitY()) * arm;
    Near(pitch.command.torque, -(0.3f + (plus.z() - minus.z()) / 0.002f),
         "tilted base IK and projected gravity reach the actual motor output");
    Near(yaw.command.torque, 0.036f, "tilted base Yaw reference");
  });
  enqueue(
      [&] {
        attitude = Eigen::Quaternionf(
            Eigen::AngleAxisf(0.15f, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(-0.2f, Eigen::Vector3f::UnitY()));
        euler = attitude.ToEulerAngleZYX();
        ai.gimbal.yaw = 0.2f;
        ai.gimbal.pit = 0.1f;
        pitch.feedback.omega = 0.4f;
        yaw.feedback.omega = 0.3f;
        publish();
      },
      [] {});
  // Ordinary aim -> operator -> auto fallback. Original profiles still use
  // gyro.
  enqueue(
      [&] {
        cmd.SetCtrlMode(CMD::Mode::CMD_OP_CTRL);
        publish();
      },
      [] {});
  enqueue(publish, [&] {
    Near(pitch.command.torque, 3.85f - 0.6f * std::sin(0.1f),
         "operator path retains IMU feedback");
    Near(yaw.command.torque, -1.08f, "operator path retains original Yaw loop");
  });
  enqueue(
      [&] {
        ai.gimbal_online = false;
        publish();
        cmd.SetCtrlMode(CMD::Mode::CMD_AUTO_CTRL);
      },
      [] {});
  enqueue(publish, [&] {
    Near(pitch.command.torque, 3.85f - 0.6f * std::sin(0.1f),
         "AI offline fallback retains original controller");
  });
  enqueue(
      [&] {
        ai.gimbal_online = true;
        attitude = LibXR::Quaternion<float>();
        euler = LibXR::EulerAngle<float>();
        yaw.feedback.abs_angle = param.yaw_zero;
        pitch.feedback.abs_angle = param.pit_zero;
        yaw.feedback.omega = pitch.feedback.omega = 0;
        publish();
      },
      [] {});
  for (int i = 0; i < 300; ++i) enqueue(publish, [] {});
  for (int i = 1; i <= 200; ++i) {
    enqueue(
        [&, i] {
          attitude = Eigen::Quaternionf(
              Eigen::AngleAxisf(0.0004f * i, Eigen::Vector3f::UnitZ()));
          euler = attitude.ToEulerAngleZYX();
          publish();
        },
        [&, i] {
          if (i == 200) {
            Near(yaw.command.torque, 0.0384f,
                 "fixed world target and rotating base generate joint velocity",
                 0.002f);
          }
        });
  }
  enqueue(
      [&] {
        gimbal.GetEvent().Active(
            static_cast<uint32_t>(GimbalEvent::SET_MODE_RELAX));
        publish();
      },
      [&] {
        Near(pitch.command.torque, 0, "RELAX pitch");
        Near(yaw.command.torque, 0, "RELAX yaw");
        if (pitch.relaxed == 0 || yaw.relaxed == 0) std::exit(1);
      });
  GimbalHost::after_tick = [&] {
    check_previous();
    if (steps.empty()) throw Complete{};
    auto step = std::move(steps.front());
    steps.pop_front();
    check_previous = std::move(step.check);
    step.setup();
  };
  try {
    GimbalHost::thread_entry();
  } catch (const Complete&) {
  }
}

void TestTorqueSlew() {
  auto euler_topic =
      LibXR::Topic::CreateTopic<LibXR::EulerAngle<float>>("gimbal_euler");
  auto gyro_topic = LibXR::Topic::CreateTopic<Eigen::Vector3f>("gimbal_gyro");
  auto quaternion_topic =
      LibXR::Topic::CreateTopic<LibXR::Quaternion<float>>("ahrs_quaternion");
  CMD cmd(CMD::Mode::CMD_AUTO_CTRL);
  FakeMotor pitch, yaw;
  auto param = Parameters();
  param.yaw_output_limits = {.torque_soft_limit_nm = 0,
                             .torque_min_nm = -0.03f,
                             .torque_max_nm = 0.08f,
                             .torque_slew_rate_nm_s = 1,
                             .torque_slew_enable = true};
  yaw.feedback.state = pitch.feedback.state = 1;
  yaw.feedback.abs_angle = param.yaw_zero;
  pitch.feedback.abs_angle = param.pit_zero;
  Gimbal gimbal(cmd, pitch, yaw, param);
  CMD::Data rc{};
  rc.chassis_online = rc.gimbal_online = true;
  cmd.FeedRC(rc);
  CMD::Data ai{};
  ai.gimbal_online = true;
  ai.gimbal.yaw = 1;
  auto publish = [&] {
    LibXR::EulerAngle<float> euler;
    Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
    LibXR::Quaternion<float> attitude;
    euler_topic.Publish(euler);
    gyro_topic.Publish(gyro);
    quaternion_topic.Publish(attitude);
    cmd.FeedAIGimbal(ai);
  };
  std::deque<Step> steps;
  auto enqueue = [&](std::function<void()> setup, std::function<void()> check) {
    steps.push_back({setup, check});
  };
  enqueue(
      [&] {
        gimbal.GetEvent().Active(
            static_cast<uint32_t>(GimbalEvent::SET_VISION_BIG_BUFF));
        publish();
      },
      [] {});
  for (int i = 0; i < 100; ++i) enqueue(publish, [] {});
  enqueue(publish,
          [&] { Near(yaw.command.torque, 0.08f, "legacy motor upper limit"); });
  enqueue(
      [&] {
        gimbal.GetEvent().Active(
            static_cast<uint32_t>(GimbalEvent::SET_VISION_AUTO_AIM));
        publish();
      },
      [&] {
        Near(yaw.command.torque, 0.079f,
             "joint reset maps actual applied motor torque");
      });
  enqueue(publish, [&] {
    Near(yaw.command.torque, 0.078f,
         "joint commit preserves motor slew direction");
  });
  for (int i = 0; i < 150; ++i) enqueue(publish, [] {});
  enqueue(publish, [&] {
    Near(yaw.command.torque, -0.03f,
         "joint limits map asymmetric motor bounds");
  });
  enqueue(
      [&] {
        gimbal.GetEvent().Active(
            static_cast<uint32_t>(GimbalEvent::SET_VISION_BIG_BUFF));
        publish();
      },
      [&] {
        Near(yaw.command.torque, -0.029f,
             "legacy reset uses actual motor torque");
      });
  enqueue(publish, [&] {
    Near(yaw.command.torque, -0.028f, "legacy slew continues after joint exit");
  });
  std::function<void()> check_previous = [] {};
  GimbalHost::after_tick = [&] {
    check_previous();
    if (steps.empty()) throw Complete{};
    auto step = std::move(steps.front());
    steps.pop_front();
    check_previous = std::move(step.check);
    step.setup();
  };
  try {
    GimbalHost::thread_entry();
  } catch (const Complete&) {
  }
}
}  // namespace

int main(int argc, char**) {
  if (argc > 1)
    TestTorqueSlew();
  else
    TestRealControlLoop();
}
