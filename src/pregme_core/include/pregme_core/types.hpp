#pragma once

#include <array>
#include <cstdint>

// =============================================================================
// pregme_core 公共类型定义
// -----------------------------------------------------------------------------
// 本头文件冻结 M1 阶段的跨包数据接口：运行模式、故障位、复位原因、输入
// 快照和控制输出。其中 Fault 的位定义必须与 eso_interfaces/msg/PregmeStatus.msg
// 中的 FAULT_* 常量逐位一致，由 config_contract.py 做一致性校验。
// 所有向量均使用固定长度 std::array，避免运行期动态分配。
// =============================================================================

namespace pregme
{

// 运行模式。数值与 ROS 参数 mode、PregmeStatus.requested_mode 一致：
//   Native=0              纯 PX4 原生控制，ESO 只观测；
//   Shadow=1              影子对比，不写控制；
//   BoundedCompensation=2 有界补偿，受安全门约束地写控制；
//   FullControl=3         完整控制。
enum class Mode : std::uint8_t {
  Native = 0,
  Shadow = 1,
  BoundedCompensation = 2,
  FullControl = 3,
};

// 故障位掩码。位号一旦发布即为接口契约，只能新增、不能改号或删除。
// 按语义可分为四组：
//   1) 输入通道故障（位置/姿态/角速度/飞控状态/分配器/参考/原生控制镜像）；
//   2) 全局健康故障（时间戳、估计器复位、非有限值、参数、PX4 失效保护）；
//   3) 主动控制前提故障（未解锁、未 Offboard、占位参数、仿真上下文、重入）；
//   4) 算法/输出放行故障（核心算法未就绪、输出总开关关闭）。
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

// 复位/纪元切换原因位。节点在估计器计数器变化或 PX4 时间戳整体回退时
// 设置这些位，并随状态和事件发布，供事后分析定位触发源。
enum ResetReason : std::uint32_t {
  ResetNone = 0,
  ResetPositionXY = 1U << 0,      // 局部位置 XY 估计被重置
  ResetPositionZ = 1U << 1,       // 局部高度估计被重置
  ResetVelocityXY = 1U << 2,      // 水平速度估计被重置
  ResetVelocityZ = 1U << 3,       // 垂直速度估计被重置
  ResetAttitude = 1U << 4,        // 姿态四元数估计被重置
  ResetPx4TimeEpoch = 1U << 5,    // PX4 时间基准切换（飞控重启/SITL 重开）
};

// 节点维护的最近一帧输入快照，主题回显与后续算法核心共用。
// 约定：位置/速度/参考量在 NED 世界系，角速度/控制量在 FRD 机体系；
// 时间戳单位为微秒，直接沿用 PX4 消息里的 timestamp/timestamp_sample。
struct InputState {
  // 各通道最近一次被接受的采样时间戳（微秒）。
  std::uint64_t local_position_timestamp_sample_us{0};
  std::uint64_t attitude_timestamp_sample_us{0};
  std::uint64_t angular_velocity_timestamp_sample_us{0};
  std::uint64_t vehicle_status_timestamp_us{0};
  std::uint64_t allocator_timestamp_us{0};
  std::uint64_t reference_timestamp_us{0};
  std::uint64_t native_thrust_timestamp_sample_us{0};
  std::uint64_t native_torque_timestamp_sample_us{0};
  // 状态估计与参考轨迹。
  std::array<float, 3> position_ned{};
  std::array<float, 3> velocity_ned{};
  std::array<float, 4> attitude_wxyz{1.F, 0.F, 0.F, 0.F};
  std::array<float, 3> angular_velocity_frd{};
  std::array<float, 3> reference_position_ned{};
  std::array<float, 3> reference_velocity_ned{};
  std::array<float, 3> reference_acceleration_ned{};
  float reference_yaw_ned{0.F};
  float reference_yawspeed_ned{0.F};
  // PX4 原生控制量镜像（Shadow 模式的对比基准）。
  std::array<float, 3> native_thrust_setpoint_frd{};
  std::array<float, 3> native_torque_setpoint_frd{};
};

// ESO 最终写给 PX4 的控制量：机体系 FRD 下的三轴推力和三轴力矩。
// valid=false 表示尚未产生可信输出（算法未接入时的常态），节点据此拒绝发布。
struct ControlOutput {
  std::array<float, 3> thrust_frd{};
  std::array<float, 3> torque_frd{};
  bool valid{false};
};

// 算法就绪硬门：这里刻意返回常量 false，不近似论文公式。
// 只有当公式追踪、黄金向量验证和测试全部落地后，才允许把它改为 true；
// source_contract 测试会检查调用点存在，防止绕过这道门。
constexpr bool algorithm_ready() noexcept {return false;}

}  // namespace pregme
