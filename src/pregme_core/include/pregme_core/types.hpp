#pragma once

#include <array>
#include <cstdint>

namespace pregme
{

enum class Mode : std::uint8_t {
  Native = 0,
  Shadow = 1,
  BoundedCompensation = 2,
  FullControl = 3,
};

enum Fault : std::uint32_t {
  FaultNone = 0,
  FaultInvalidMode = 1U << 0,
  FaultLocalPosition = 1U << 1,
  FaultAttitude = 1U << 2,
  FaultAngularVelocity = 1U << 3,
  FaultVehicleStatus = 1U << 4,
  FaultAllocatorStatus = 1U << 5,
  FaultTimestamp = 1U << 6,
  FaultEstimatorReset = 1U << 7,
  FaultNonFinite = 1U << 8,
  FaultAlgorithmUnavailable = 1U << 9,
  FaultOutputDisabled = 1U << 10,
  FaultReference = 1U << 11,
  FaultNativeThrust = 1U << 12,
  FaultNativeTorque = 1U << 13,
  FaultParameterInvalid = 1U << 14,
  FaultPlaceholderParameters = 1U << 15,
  FaultSimulationContext = 1U << 16,
  FaultNotArmed = 1U << 17,
  FaultNotOffboard = 1U << 18,
  FaultFailsafe = 1U << 19,
  FaultReentryRequired = 1U << 20,
};

enum ResetReason : std::uint32_t {
  ResetNone = 0,
  ResetPositionXY = 1U << 0,
  ResetPositionZ = 1U << 1,
  ResetVelocityXY = 1U << 2,
  ResetVelocityZ = 1U << 3,
  ResetAttitude = 1U << 4,
  ResetPx4TimeEpoch = 1U << 5,
};

struct InputState {
  std::uint64_t local_position_timestamp_sample_us{0};
  std::uint64_t attitude_timestamp_sample_us{0};
  std::uint64_t angular_velocity_timestamp_sample_us{0};
  std::uint64_t vehicle_status_timestamp_us{0};
  std::uint64_t allocator_timestamp_us{0};
  std::uint64_t reference_timestamp_us{0};
  std::uint64_t native_thrust_timestamp_sample_us{0};
  std::uint64_t native_torque_timestamp_sample_us{0};
  std::array<float, 3> position_ned{};
  std::array<float, 3> velocity_ned{};
  std::array<float, 4> attitude_wxyz{1.F, 0.F, 0.F, 0.F};
  std::array<float, 3> angular_velocity_frd{};
  std::array<float, 3> reference_position_ned{};
  std::array<float, 3> reference_velocity_ned{};
  std::array<float, 3> reference_acceleration_ned{};
  float reference_yaw_ned{0.F};
  float reference_yawspeed_ned{0.F};
  std::array<float, 3> native_thrust_setpoint_frd{};
  std::array<float, 3> native_torque_setpoint_frd{};
};

struct ControlOutput {
  std::array<float, 3> thrust_frd{};
  std::array<float, 3> torque_frd{};
  bool valid{false};
};

// The paper equations are intentionally not approximated here. This turns true only
// after formula traceability and golden-vector validation are implemented.
constexpr bool algorithm_ready() noexcept {return false;}

}  // namespace pregme
