#pragma once

#include <array>
#include <cstdint>

namespace pregme
{

using Vector3d = std::array<double, 3>;
using Matrix3dRowMajor = std::array<double, 9>;

enum class ParameterStatus : std::uint8_t {
  Reported = 0,
  Measured = 1,
  Placeholder = 2,
  PermanentlyMissing = 3,
};

enum class IntegrationMethod : std::uint8_t {
  RungeKutta4 = 0,
};

struct ModelParameters {
  double body_mass_kg{0.0};
  double arm_mass_kg{0.0};
  Matrix3dRowMajor inertia_body_kg_m2{};
  ParameterStatus inertia_status{ParameterStatus::PermanentlyMissing};
};

struct NumericsPolicy {
  IntegrationMethod integration_method{IntegrationMethod::RungeKutta4};
  double expected_position_period_s{0.0};
  double expected_attitude_period_s{0.0};
  double minimum_period_s{0.0};
  double maximum_period_s{0.0};
  double desired_attitude_derivative_cutoff_hz{0.0};
};

struct PositionReference {
  Vector3d position_ned_m{};
  Vector3d velocity_ned_m_s{};
  Vector3d acceleration_ned_m_s2{};
  double yaw_ned_rad{0.0};
  double yawspeed_ned_rad_s{0.0};
  std::uint64_t timestamp_us{0};
};

struct NativeControlObservation {
  Vector3d thrust_frd_normalized{};
  Vector3d torque_frd_normalized{};
  std::uint64_t thrust_timestamp_sample_us{0};
  std::uint64_t torque_timestamp_sample_us{0};
};

// 本头文件只冻结 M1 数据接口。M2/M3 通过黄金向量后才能加入公式实现。
struct AlgorithmContract {
  ModelParameters model{};
  NumericsPolicy numerics{};
  PositionReference reference{};
  NativeControlObservation native_control{};
  bool has_placeholder{false};
};

}  // namespace pregme
