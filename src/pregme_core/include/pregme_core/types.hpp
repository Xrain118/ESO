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
};

struct InputState {
  std::uint64_t timestamp_sample_us{0};
  std::array<float, 3> position_ned{};
  std::array<float, 3> velocity_ned{};
  std::array<float, 4> attitude_wxyz{1.F, 0.F, 0.F, 0.F};
  std::array<float, 3> angular_velocity_frd{};
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
