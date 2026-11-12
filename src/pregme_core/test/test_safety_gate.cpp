#include <gtest/gtest.h>

#include "pregme_core/safety_gate.hpp"

namespace
{

pregme::SafetyInputs valid_inputs()
{
  pregme::SafetyInputs inputs{};
  inputs.now_ns = 1000000000;
  inputs.local_position_received_ns = 900000000;
  inputs.attitude_received_ns = 900000000;
  inputs.angular_velocity_received_ns = 900000000;
  inputs.vehicle_status_received_ns = 900000000;
  inputs.allocator_status_received_ns = 900000000;
  inputs.local_position_valid = true;
  inputs.attitude_valid = true;
  inputs.angular_velocity_valid = true;
  inputs.vehicle_status_valid = true;
  inputs.allocator_status_valid = true;
  inputs.values_finite = true;
  return inputs;
}

TEST(SafetyGate, NativeAndShadowNeverPublishControl)
{
  pregme::SafetyGate gate;
  EXPECT_FALSE(gate.evaluate(0, true, true, valid_inputs()).publish_control);
  EXPECT_FALSE(gate.evaluate(1, true, true, valid_inputs()).publish_control);
}

TEST(SafetyGate, ActiveModesRequireBothHardGates)
{
  pregme::SafetyGate gate;
  EXPECT_FALSE(gate.evaluate(2, false, true, valid_inputs()).publish_control);
  EXPECT_FALSE(gate.evaluate(2, true, false, valid_inputs()).publish_control);
  EXPECT_TRUE(gate.evaluate(2, true, true, valid_inputs()).publish_control);
}

TEST(SafetyGate, StaleStateAndResetForceNative)
{
  pregme::SafetyGate gate;
  auto stale = valid_inputs();
  stale.local_position_received_ns = 1;
  const auto stale_decision = gate.evaluate(3, true, true, stale);
  EXPECT_FALSE(stale_decision.publish_control);
  EXPECT_NE(stale_decision.fault_flags & pregme::FaultLocalPosition, 0U);

  auto reset = valid_inputs();
  reset.estimator_reset = true;
  const auto reset_decision = gate.evaluate(3, true, true, reset);
  EXPECT_FALSE(reset_decision.publish_control);
  EXPECT_NE(reset_decision.fault_flags & pregme::FaultEstimatorReset, 0U);
}

TEST(SafetyGate, InvalidModeIsRejected)
{
  pregme::SafetyGate gate;
  const auto decision = gate.evaluate(9, true, true, valid_inputs());
  EXPECT_EQ(decision.active_mode, pregme::Mode::Native);
  EXPECT_NE(decision.fault_flags & pregme::FaultInvalidMode, 0U);
}

}  // namespace
