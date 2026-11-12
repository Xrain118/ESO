#include "pregme_core/safety_gate.hpp"

namespace pregme
{

bool SafetyGate::fresh(
  const std::int64_t now_ns, const std::int64_t received_ns,
  const std::int64_t timeout_ns) const noexcept
{
  return received_ns > 0 && now_ns >= received_ns && now_ns - received_ns <= timeout_ns;
}

SafetyDecision SafetyGate::evaluate(
  const std::int64_t requested_mode, const bool enable_control_output,
  const bool core_algorithm_ready, const SafetyInputs & inputs) const noexcept
{
  SafetyDecision decision{};

  if (requested_mode < static_cast<std::int64_t>(Mode::Native) ||
    requested_mode > static_cast<std::int64_t>(Mode::FullControl))
  {
    decision.fault_flags |= FaultInvalidMode;
    return decision;
  }

  decision.requested_mode = static_cast<Mode>(requested_mode);

  if (!inputs.local_position_valid ||
    !fresh(inputs.now_ns, inputs.local_position_received_ns, config_.state_timeout_ns))
  {
    decision.fault_flags |= FaultLocalPosition;
  }
  if (!inputs.attitude_valid ||
    !fresh(inputs.now_ns, inputs.attitude_received_ns, config_.state_timeout_ns))
  {
    decision.fault_flags |= FaultAttitude;
  }
  if (!inputs.angular_velocity_valid ||
    !fresh(inputs.now_ns, inputs.angular_velocity_received_ns, config_.state_timeout_ns))
  {
    decision.fault_flags |= FaultAngularVelocity;
  }
  if (!inputs.vehicle_status_valid ||
    !fresh(inputs.now_ns, inputs.vehicle_status_received_ns, config_.state_timeout_ns))
  {
    decision.fault_flags |= FaultVehicleStatus;
  }
  if (!inputs.allocator_status_valid ||
    !fresh(inputs.now_ns, inputs.allocator_status_received_ns, config_.allocator_timeout_ns))
  {
    decision.fault_flags |= FaultAllocatorStatus;
  }
  if (!inputs.timestamp_monotonic) {
    decision.fault_flags |= FaultTimestamp;
  }
  if (inputs.estimator_reset) {
    decision.fault_flags |= FaultEstimatorReset;
  }
  if (!inputs.values_finite) {
    decision.fault_flags |= FaultNonFinite;
  }

  decision.inputs_valid = decision.fault_flags == FaultNone;

  if (decision.requested_mode == Mode::Native || decision.requested_mode == Mode::Shadow) {
    decision.active_mode = decision.requested_mode;
    return decision;
  }

  if (!core_algorithm_ready) {
    decision.fault_flags |= FaultAlgorithmUnavailable;
  }
  if (!enable_control_output) {
    decision.fault_flags |= FaultOutputDisabled;
  }

  decision.inputs_valid = decision.fault_flags == FaultNone;
  if (decision.inputs_valid) {
    decision.active_mode = decision.requested_mode;
    decision.publish_control = true;
  }
  return decision;
}

}  // namespace pregme
