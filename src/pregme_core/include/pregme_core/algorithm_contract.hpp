#pragma once

#include <array>
#include <cstdint>

// =============================================================================
// AlgorithmContract：M1 冻结的算法数据接口
// -----------------------------------------------------------------------------
// 本头文件只定义「算法需要什么数据」，不包含任何公式实现。M2/M3 只有在
// 公式追踪表和黄金向量验证通过后，才能在此基础上增加实现。
// 数值统一使用双精度（double），坐标系统一约定：
//   NED  世界系（位置/速度/加速度/偏航）；
//   FRD  机体系（推力/力矩）；
// 所有量都带单位后缀，避免量纲混淆。
// =============================================================================

namespace pregme
{

using Vector3d = std::array<double, 3>;
// 3x3 矩阵按行优先展开为 9 个元素：{m00,m01,m02,m10,...,m22}。
using Matrix3dRowMajor = std::array<double, 9>;

// 参数来源的可信度分级，用于区分「论文报告/实测/占位/永久缺失」，
// 与 config/m2_fixture_metadata.yaml 中的 status 字段语义对应。
enum class ParameterStatus : std::uint8_t {
  Reported = 0,
  Measured = 1,
  Placeholder = 2,
  PermanentlyMissing = 3,
};

// 数值积分策略。当前只冻结四阶 Runge-Kutta，后续新增方法需同步更新契约。
enum class IntegrationMethod : std::uint8_t {
  RungeKutta4 = 0,
};

// 刚体模型参数：机体质量、机械臂质量、机体系对角惯量及其来源状态。
struct ModelParameters {
  double body_mass_kg{0.0};
  double arm_mass_kg{0.0};
  Matrix3dRowMajor inertia_body_kg_m2{};
  ParameterStatus inertia_status{ParameterStatus::PermanentlyMissing};
};

// 数值与时序策略：积分方法、两路参考的期望周期、允许周期窗口，
// 以及期望姿态导数的低通截止频率。数值必须与参数集（夹具/实机）一致。
struct NumericsPolicy {
  IntegrationMethod integration_method{IntegrationMethod::RungeKutta4};
  double expected_position_period_s{0.0};
  double expected_attitude_period_s{0.0};
  double minimum_period_s{0.0};
  double maximum_period_s{0.0};
  double desired_attitude_derivative_cutoff_hz{0.0};
};

// 位置参考轨迹（NED）：位置、速度、加速度、偏航角/角速度及采样时间戳。
struct PositionReference {
  Vector3d position_ned_m{};
  Vector3d velocity_ned_m_s{};
  Vector3d acceleration_ned_m_s2{};
  double yaw_ned_rad{0.0};
  double yawspeed_ned_rad_s{0.0};
  std::uint64_t timestamp_us{0};
};

// PX4 原生控制量观测（FRD，归一化），作为影子模式的对比基准；
// 推力与力矩各自携带采样时间戳，便于对齐和延迟分析。
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
