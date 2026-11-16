// =============================================================================
// SafetyGate / TimestampTracker / SimulationClockGuard 单元测试
// -----------------------------------------------------------------------------
// 目标：用最小的可控输入覆盖安全门的每个放行条件和故障分支，锁定 M1 冻结的
//       接口行为。所有用例都不依赖 ROS 运行时。
// =============================================================================

#include <gtest/gtest.h>

#include "pregme_core/safety_gate.hpp"
#include "pregme_core/simulation_clock_guard.hpp"
#include "pregme_core/timestamp_tracker.hpp"

namespace
{

// 构造一份「全部健康」的输入基线：所有通道有效、未超时、已解锁且 Offboard。
// 各用例在此基础上只改动需要验证的字段，保持测试意图单一。
pregme::SafetyInputs valid_inputs()
{
  pregme::SafetyInputs inputs{};
  inputs.now_ns = 1000000000;
  inputs.local_position_received_ns = 900000000;
  inputs.attitude_received_ns = 900000000;
  inputs.angular_velocity_received_ns = 900000000;
  inputs.vehicle_status_received_ns = 900000000;
  inputs.allocator_status_received_ns = 900000000;
  inputs.reference_received_ns = 900000000;
  inputs.native_thrust_received_ns = 900000000;
  inputs.native_torque_received_ns = 900000000;
  inputs.local_position_valid = true;
  inputs.attitude_valid = true;
  inputs.angular_velocity_valid = true;
  inputs.vehicle_status_valid = true;
  inputs.allocator_status_valid = true;
  inputs.reference_valid = true;
  inputs.native_thrust_valid = true;
  inputs.native_torque_valid = true;
  inputs.values_finite = true;
  inputs.armed = true;
  inputs.offboard = true;
  return inputs;
}

// 即使输入完全健康、算法就绪、输出开关打开，Native/Shadow 也不得写控制。
TEST(SafetyGate, NativeAndShadowNeverPublishControl)
{
  pregme::SafetyGate gate;
  EXPECT_FALSE(gate.evaluate(0, true, true, valid_inputs()).publish_control);
  EXPECT_FALSE(gate.evaluate(1, true, true, valid_inputs()).publish_control);
}

// 主动模式的两个硬门：输出开关与核心算法就绪，任一关闭都不得发布控制。
TEST(SafetyGate, ActiveModesRequireBothHardGates)
{
  pregme::SafetyGate gate;
  EXPECT_FALSE(gate.evaluate(2, false, true, valid_inputs()).publish_control);
  EXPECT_FALSE(gate.evaluate(2, true, false, valid_inputs()).publish_control);
  EXPECT_TRUE(gate.evaluate(2, true, true, valid_inputs()).publish_control);
}

// 输入超时和估计器复位都应阻止主动控制，并各自置对应故障位。
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

// 越界模式必须被拒绝，并回落到 Native。
TEST(SafetyGate, InvalidModeIsRejected)
{
  pregme::SafetyGate gate;
  const auto decision = gate.evaluate(9, true, true, valid_inputs());
  EXPECT_EQ(decision.active_mode, pregme::Mode::Native);
  EXPECT_NE(decision.fault_flags & pregme::FaultInvalidMode, 0U);
}

// Shadow 模式需要参考轨迹和原生推力/力矩镜像；缺失时三项故障都要置位。
TEST(SafetyGate, ShadowRequiresReferenceAndNativeControlObservations)
{
  pregme::SafetyGate gate;
  auto inputs = valid_inputs();
  inputs.reference_valid = false;
  inputs.native_thrust_valid = false;
  inputs.native_torque_valid = false;
  const auto decision = gate.evaluate(1, false, false, inputs);
  EXPECT_FALSE(decision.inputs_valid);
  EXPECT_NE(decision.fault_flags & pregme::FaultReference, 0U);
  EXPECT_NE(decision.fault_flags & pregme::FaultNativeThrust, 0U);
  EXPECT_NE(decision.fault_flags & pregme::FaultNativeTorque, 0U);
  EXPECT_FALSE(decision.publish_control);
}

// 占位参数进入主动控制的三级放行：默认禁止 -> 需 SITL 显式许可 ->
// 还需仿真时钟对齐；三者逐级满足后才允许发布控制。
TEST(SafetyGate, PlaceholderRequiresVerifiedSimulationContext)
{
  pregme::SafetyGate gate;
  auto inputs = valid_inputs();
  inputs.has_placeholder = true;
  auto decision = gate.evaluate(2, true, true, inputs);
  EXPECT_NE(decision.fault_flags & pregme::FaultPlaceholderParameters, 0U);

  inputs.placeholder_allowed = true;
  decision = gate.evaluate(2, true, true, inputs);
  EXPECT_NE(decision.fault_flags & pregme::FaultSimulationContext, 0U);

  inputs.simulation_context_valid = true;
  decision = gate.evaluate(2, true, true, inputs);
  EXPECT_TRUE(decision.publish_control);
}

// 时间戳跟踪器的状态迁移：首帧 -> 重复 -> 回退（锁存）-> 恢复 -> 归零（锁存）。
TEST(TimestampTracker, DuplicateRegressionAndRecoveryAreExplicit)
{
  pregme::TimestampTracker tracker;
  EXPECT_EQ(tracker.observe(100U), pregme::TimestampResult::First);
  EXPECT_EQ(tracker.observe(100U), pregme::TimestampResult::Duplicate);
  EXPECT_EQ(tracker.observe(99U), pregme::TimestampResult::Regression);
  EXPECT_TRUE(tracker.fault_latched());
  EXPECT_EQ(tracker.observe(101U), pregme::TimestampResult::Advanced);
  EXPECT_FALSE(tracker.fault_latched());
  EXPECT_EQ(tracker.observe(0U), pregme::TimestampResult::InvalidZero);
  EXPECT_TRUE(tracker.fault_latched());
}

// 仿真时钟守卫必须同时满足：use_sim_time、显式许可、偏移稳定、观察新鲜。
// 用例依次关闭每个条件，验证 valid() 都会失败。
TEST(SimulationClockGuard, RequiresStableOffsetFreshnessAndBothPermissions)
{
  pregme::SimulationClockGuard guard;
  EXPECT_TRUE(guard.observe(1000000000LL, 5000000U, 100LL, 100000000LL));
  EXPECT_FALSE(guard.valid(110LL, 200LL, false, true));
  EXPECT_FALSE(guard.valid(110LL, 200LL, true, false));
  EXPECT_TRUE(guard.valid(110LL, 200LL, true, true));
  EXPECT_TRUE(guard.observe(1100000000LL, 5100000U, 120LL, 100000000LL));
  EXPECT_FALSE(guard.observe(1200000000LL, 5400000U, 140LL, 100000000LL));
  EXPECT_FALSE(guard.valid(150LL, 200LL, true, true));
}

}  // namespace
