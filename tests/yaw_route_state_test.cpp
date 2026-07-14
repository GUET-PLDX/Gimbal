#include "yaw_lqr_eso_test_support.hpp"

static YawRouteState::Input ready_input() {
  return {.route_enable = false,
          .ai_source = false,
          .reference_valid = true,
          .controller_config_valid = true,
          .feedback_valid = true,
          .dt_valid = true,
          .gimbal_control_enabled = true,
          .yaw_torque_submission_ready = true,
          .cmd_sample_seq = 10U};
}

static YawRouteState::Input enter_lqr(YawRouteState& route) {
  auto input = ready_input();
  auto decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LEGACY_RUN);

  input.route_enable = true;
  input.ai_source = true;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);

  ++input.cmd_sample_seq;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  return input;
}

static void test_source_barrier_and_rearm() {
  YawRouteState route;
  auto input = ready_input();
  auto decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LEGACY_RUN);
  CHECK(!decision.action_changed);
  CHECK(!decision.entered_lqr);
  CHECK(!decision.exited_lqr);

  input.route_enable = true;
  input.ai_source = true;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);
  CHECK(decision.action_changed);
  CHECK(!decision.entered_lqr);
  CHECK(!decision.exited_lqr);
  CHECK(decision.rearm_pending);

  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);
  CHECK(!decision.action_changed);

  input.cmd_sample_seq = 9U;
  CHECK(route.Step(input).action == YawRouteState::Action::HOLD_CURRENT);

  input.cmd_sample_seq = 11U;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  CHECK(decision.action_changed);
  CHECK(decision.entered_lqr);
  CHECK(!decision.exited_lqr);
  CHECK(decision.rearm_pending);

  route.ConfirmLqrCommit();
  CHECK(!route.RearmPending());
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  CHECK(!decision.action_changed);
  CHECK(!decision.entered_lqr);
  CHECK(!decision.exited_lqr);
  CHECK(!decision.rearm_pending);

  input.dt_valid = false;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::ZERO_OUTPUT);
  CHECK(decision.action_changed);
  CHECK(!decision.entered_lqr);
  CHECK(decision.exited_lqr);
  CHECK(decision.rearm_pending);

  input.dt_valid = true;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  CHECK(decision.entered_lqr);
  CHECK(decision.rearm_pending);
  route.ConfirmLqrCommit();
  CHECK(!route.RearmPending());
}

static void test_route_off_behavior() {
  YawRouteState route;
  auto input = enter_lqr(route);
  route.ConfirmLqrCommit();
  CHECK(!route.RearmPending());

  input.route_enable = false;
  auto decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LEGACY_RUN);
  CHECK(decision.action_changed);
  CHECK(decision.exited_lqr);
  CHECK(!decision.rearm_pending);

  input.controller_config_valid = false;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LEGACY_RUN);
  CHECK(!decision.action_changed);
  CHECK(!decision.rearm_pending);

  input.reference_valid = false;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);
  CHECK(decision.rearm_pending);

  input.reference_valid = true;
  input.controller_config_valid = true;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LEGACY_RUN);

  input.route_enable = true;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);
  CHECK(decision.rearm_pending);
  CHECK(route.Step(input).action == YawRouteState::Action::HOLD_CURRENT);

  ++input.cmd_sample_seq;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  CHECK(decision.entered_lqr);
}

static void test_source_falling_barrier() {
  YawRouteState route;
  auto input = enter_lqr(route);
  route.ConfirmLqrCommit();

  input.ai_source = false;
  auto decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);
  CHECK(decision.action_changed);
  CHECK(decision.exited_lqr);
  CHECK(decision.rearm_pending);

  CHECK(route.Step(input).action == YawRouteState::Action::HOLD_CURRENT);
  --input.cmd_sample_seq;
  CHECK(route.Step(input).action == YawRouteState::Action::HOLD_CURRENT);

  input.cmd_sample_seq += 2U;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LEGACY_RUN);
  CHECK(decision.action_changed);

  input.ai_source = true;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);
  CHECK(decision.action_changed);
  CHECK(route.Step(input).action == YawRouteState::Action::HOLD_CURRENT);

  ++input.cmd_sample_seq;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  CHECK(decision.entered_lqr);
}

static void test_invalid_input_actions() {
  YawRouteState route;
  auto input = enter_lqr(route);
  route.ConfirmLqrCommit();

  input.controller_config_valid = false;
  auto decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);
  CHECK(decision.exited_lqr);
  CHECK(decision.rearm_pending);

  input.controller_config_valid = true;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  CHECK(decision.entered_lqr);
  CHECK(decision.rearm_pending);

  input.reference_valid = false;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);
  CHECK(decision.rearm_pending);

  input.reference_valid = true;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  CHECK(decision.entered_lqr);
}

static void test_relax_and_zero_precedence() {
  YawRouteState route;
  auto input = enter_lqr(route);
  route.ConfirmLqrCommit();

  input.dt_valid = false;
  input.reference_valid = false;
  auto decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::ZERO_OUTPUT);
  CHECK(decision.rearm_pending);

  input.dt_valid = true;
  input.feedback_valid = false;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::RELAX);
  CHECK(decision.rearm_pending);

  input.feedback_valid = true;
  input.gimbal_control_enabled = false;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::RELAX);
  CHECK(!decision.action_changed);
  CHECK(decision.rearm_pending);

  input.gimbal_control_enabled = true;
  input.reference_valid = true;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  CHECK(decision.entered_lqr);
  CHECK(decision.rearm_pending);
}

static void test_motor_submission_gate() {
  YawRouteState route;
  auto input = enter_lqr(route);
  route.ConfirmLqrCommit();

  input.yaw_torque_submission_ready = false;
  auto decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);
  CHECK(decision.exited_lqr);
  CHECK(decision.rearm_pending);
  route.ConfirmLqrCommit();
  CHECK(route.RearmPending());

  input.yaw_torque_submission_ready = true;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  CHECK(decision.entered_lqr);
  CHECK(decision.rearm_pending);
  route.ConfirmLqrCommit();
  CHECK(!route.RearmPending());
}

static void test_rearm_confirmation() {
  YawRouteState route;
  CHECK(route.RearmPending());
  route.ConfirmLqrCommit();
  CHECK(route.RearmPending());

  auto input = enter_lqr(route);
  route.ConfirmLqrCommit();
  CHECK(!route.RearmPending());

  route.RequestRearm();
  CHECK(route.RearmPending());
  input.controller_config_valid = false;
  auto decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);
  route.ConfirmLqrCommit();
  CHECK(route.RearmPending());

  input.controller_config_valid = true;
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  CHECK(decision.rearm_pending);
  route.ConfirmLqrCommit();
  CHECK(!route.RearmPending());

  route.Reset();
  CHECK(route.RearmPending());
  route.ConfirmLqrCommit();
  CHECK(route.RearmPending());
  input = ready_input();
  decision = route.Step(input);
  CHECK(decision.action == YawRouteState::Action::LEGACY_RUN);
  CHECK(!decision.action_changed);
}

int main() {
  test_source_barrier_and_rearm();
  test_route_off_behavior();
  test_source_falling_barrier();
  test_invalid_input_actions();
  test_relax_and_zero_precedence();
  test_motor_submission_gate();
  test_rearm_confirmation();
  return yaw_test_failures == 0 ? 0 : 1;
}
