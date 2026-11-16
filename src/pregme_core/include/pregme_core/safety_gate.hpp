#pragma once

#include <cstdint>

#include "pregme_core/types.hpp"

namespace pregme
{

struct SafetyInputs {
  std::int64_t now_ns{0};
  std::int64_t local_position_received_ns{0};
  std::int64_t attitude_received_ns{0};
  std::int64_t angular_velocity_received_ns{0};
  std::int64_t vehicle_status_received_ns{0};
  std::int64_t allocator_status_received_ns{0};
  std::int64_t reference_received_ns{0};
  std::int64_t native_thrust_received_ns{0};
  std::int64_t native_torque_received_ns{0};
  bool local_position_valid{false};
  bool attitude_valid{false};
  bool angular_velocity_valid{false};
  bool vehicle_status_valid{false};
  bool allocator_status_valid{false};
  bool reference_valid{false};
  bool native_thrust_valid{false};
  bool native_torque_valid{false};
  bool timestamp_monotonic{true};
  bool estimator_reset{false};
  bool values_finite{false};
  bool armed{false};
  bool offboard{false};
  bool failsafe{false};
  bool parameters_valid{true};
  bool has_placeholder{false};
  bool placeholder_allowed{false};
  bool simulation_context_valid{false};
  bool reentry_required{false};
};

struct SafetyConfig {
  std::int64_t state_timeout_ns{200000000};
  std::int64_t allocator_timeout_ns{250000000};
  std::int64_t reference_timeout_ns{200000000};
  std::int64_t native_control_timeout_ns{200000000};
};

struct SafetyDecision {
  Mode requested_mode{Mode::Native};
  Mode active_mode{Mode::Native};
  std::uint32_t fault_flags{FaultNone};
  bool inputs_valid{false};
  bool publish_control{false};
};

class SafetyGate
{
public:
  explicit SafetyGate(SafetyConfig config = {}) noexcept : config_(config) {}

  SafetyDecision evaluate(
    std::int64_t requested_mode, bool enable_control_output,
    bool core_algorithm_ready, const SafetyInputs & inputs) const noexcept;

private:
  bool fresh(std::int64_t now_ns, std::int64_t received_ns, std::int64_t timeout_ns) const noexcept;
  SafetyConfig config_;
};

}  // namespace pregme
