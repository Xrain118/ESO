#include "pregme_control/runtime_config.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>

namespace pregme_control
{
namespace
{

bool absolute_topic(const std::string & topic)
{
  return topic.size() > 1U && topic.front() == '/' && topic.find(' ') == std::string::npos;
}

bool lower_hex_sha256(const std::string & value)
{
  return value.size() == 64U && std::all_of(
    value.begin(), value.end(), [](const unsigned char character) {
      return std::isdigit(character) != 0 || (character >= 'a' && character <= 'f');
    });
}

}  // namespace

ConfigValidation validate_runtime_config(const RuntimeConfig & config)
{
  if (config.mode < 0 || config.mode > 3) {
    return {false, "mode must be in [0, 3]"};
  }
  if (!std::isfinite(config.status_rate_hz) || config.status_rate_hz < 1.0 ||
    config.status_rate_hz > 100.0)
  {
    return {false, "status_rate_hz must be finite and in [1, 100]"};
  }
  if (config.state_timeout_ms < 20 || config.state_timeout_ms > 1000) {
    return {false, "state_timeout_ms must be in [20, 1000]"};
  }
  if (config.allocator_timeout_ms < 20 || config.allocator_timeout_ms > 1000) {
    return {false, "allocator_timeout_ms must be in [20, 1000]"};
  }
  if (config.simulation_clock_timeout_ms < 20 || config.simulation_clock_timeout_ms > 1000) {
    return {false, "simulation_clock_timeout_ms must be in [20, 1000]"};
  }
  if (config.simulation_time_tolerance_ms < 10 || config.simulation_time_tolerance_ms > 1000) {
    return {false, "simulation_time_tolerance_ms must be in [10, 1000]"};
  }

  const std::array<const std::string *, 8> topics{
    &config.local_position_topic, &config.attitude_topic, &config.angular_velocity_topic,
    &config.vehicle_status_topic, &config.allocator_status_topic, &config.reference_topic,
    &config.native_thrust_topic, &config.native_torque_topic};
  if (!std::all_of(topics.begin(), topics.end(), [](const std::string * topic) {
      return absolute_topic(*topic);
    }))
  {
    return {false, "all PX4 topics must be non-empty absolute ROS topic names"};
  }
  const bool fixture_values_present = config.body_mass_kg != 0.0 || config.arm_mass_kg != 0.0 ||
    config.wheelbase_m != 0.0 || config.inertia_xx_kg_m2 != 0.0 ||
    config.inertia_yy_kg_m2 != 0.0 || config.inertia_zz_kg_m2 != 0.0 ||
    config.expected_position_period_s != 0.0 || config.expected_attitude_period_s != 0.0 ||
    config.minimum_period_s != 0.0 || config.maximum_period_s != 0.0 ||
    config.desired_attitude_derivative_cutoff_hz != 0.0;
  const bool fixture_identity_present = config.fixture_id != "none" ||
    config.parameter_set_sha256 != "none" || config.fixture_watermark != "none";
  const bool parameter_set_present =
    config.has_placeholder || fixture_values_present || fixture_identity_present;
  if (parameter_set_present) {
    if (config.fixture_id.empty() || config.fixture_id == "none") {
      return {false, "non-default parameter sets require fixture_id"};
    }
    if (!lower_hex_sha256(config.parameter_set_sha256)) {
      return {false, "non-default parameter sets require a lowercase SHA-256"};
    }
    const auto expected_watermark =
      "FIXTURE:" + config.fixture_id + ":" + config.parameter_set_sha256.substr(0, 12);
    if (config.fixture_watermark != expected_watermark) {
      return {false, "placeholder parameter set watermark does not match its ID and SHA-256"};
    }
    if (!std::isfinite(config.body_mass_kg) || config.body_mass_kg <= 0.0 ||
      !std::isfinite(config.arm_mass_kg) || config.arm_mass_kg < 0.0 ||
      !std::isfinite(config.wheelbase_m) || config.wheelbase_m <= 0.0)
    {
      return {false, "fixture mass and geometry must be finite and positive"};
    }
    const std::array<double, 3> inertia{
      config.inertia_xx_kg_m2, config.inertia_yy_kg_m2, config.inertia_zz_kg_m2};
    for (const double value : inertia) {
      if (!std::isfinite(value) || value <= 0.0) {
        return {false, "fixture diagonal inertia must be finite and positive"};
      }
      const double radius_of_gyration = std::sqrt(value / config.body_mass_kg);
      if (radius_of_gyration < 0.1 || radius_of_gyration > 0.4) {
        return {false, "fixture radius of gyration must be in [0.1, 0.4] m"};
      }
    }
    if (!std::isfinite(config.expected_position_period_s) ||
      !std::isfinite(config.expected_attitude_period_s) ||
      !std::isfinite(config.minimum_period_s) || !std::isfinite(config.maximum_period_s) ||
      config.minimum_period_s <= 0.0 || config.maximum_period_s < config.minimum_period_s ||
      config.expected_position_period_s < config.minimum_period_s ||
      config.expected_position_period_s > config.maximum_period_s ||
      config.expected_attitude_period_s < config.minimum_period_s ||
      config.expected_attitude_period_s > config.maximum_period_s)
    {
      return {false, "fixture sample periods are inconsistent"};
    }
    if (!std::isfinite(config.desired_attitude_derivative_cutoff_hz) ||
      config.desired_attitude_derivative_cutoff_hz <= 0.0)
    {
      return {false, "fixture derivative cutoff must be finite and positive"};
    }
  }
  return {true, "ok"};
}

bool is_runtime_mutable_parameter(const std::string & name) noexcept
{
  return name == "mode" || name == "enable_control_output";
}

}  // namespace pregme_control
