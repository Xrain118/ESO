#pragma once

#include <cstdint>
#include <string>

namespace pregme_control
{

struct RuntimeConfig {
  std::int64_t mode{0};
  bool enable_control_output{false};
  double status_rate_hz{50.0};
  std::int64_t state_timeout_ms{200};
  std::int64_t allocator_timeout_ms{250};
  bool has_placeholder{false};
  bool allow_placeholder_in_sitl{false};
  std::int64_t simulation_clock_timeout_ms{200};
  std::int64_t simulation_time_tolerance_ms{100};
  std::string fixture_id{"none"};
  std::string parameter_set_sha256{"none"};
  std::string fixture_watermark{"none"};
  double body_mass_kg{0.0};
  double arm_mass_kg{0.0};
  double wheelbase_m{0.0};
  double inertia_xx_kg_m2{0.0};
  double inertia_yy_kg_m2{0.0};
  double inertia_zz_kg_m2{0.0};
  double expected_position_period_s{0.0};
  double expected_attitude_period_s{0.0};
  double minimum_period_s{0.0};
  double maximum_period_s{0.0};
  double desired_attitude_derivative_cutoff_hz{0.0};
  std::string local_position_topic{"/fmu/out/vehicle_local_position"};
  std::string attitude_topic{"/fmu/out/vehicle_attitude"};
  std::string angular_velocity_topic{"/fmu/out/vehicle_angular_velocity"};
  std::string vehicle_status_topic{"/fmu/out/vehicle_status_v1"};
  std::string allocator_status_topic{"/fmu/out/control_allocator_status"};
  std::string reference_topic{"/fmu/out/vehicle_local_position_setpoint"};
  std::string native_thrust_topic{"/fmu/out/vehicle_thrust_setpoint"};
  std::string native_torque_topic{"/fmu/out/vehicle_torque_setpoint"};
};

struct ConfigValidation {
  bool valid{false};
  std::string reason{};
};

ConfigValidation validate_runtime_config(const RuntimeConfig & config);
bool is_runtime_mutable_parameter(const std::string & name) noexcept;

}  // namespace pregme_control
