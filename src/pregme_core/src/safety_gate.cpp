#include "pregme_core/safety_gate.hpp"

// =============================================================================
// SafetyGate：纯函数式安全门控
// -----------------------------------------------------------------------------
// 角色：把「请求模式 + 各输入通道健康度 + 起飞前提」翻译为最终的模式决策和
//       故障位。本类不持有状态、不做 I/O，因此可被单元测试直接覆盖。
// 规则：
//   1. 故障位只累加（按位或），不做清除；任一故障都会阻止主动控制。
//   2. 基础状态通道在所有模式下都检查；参考轨迹和原生推力/力矩镜像只在
//      Shadow 及更高模式检查，因为 Native 模式不使用它们。
//   3. 主动模式（2=BoundedCompensation / 3=FullControl）额外要求解锁、
//      Offboard、无失效保护、参数合法、无重入、占位参数策略允许，以及
//      核心算法就绪和输出开关已打开。全部满足才置 publish_control。
// =============================================================================

namespace pregme
{

// 判断某个输入通道在 now_ns 时刻是否仍然新鲜。
// 三个条件缺一不可：
//   - received_ns > 0：0 是「从未收到」的哨兵值；
//   - now_ns >= received_ns：防止时钟回退导致的时间差为负；
//   - now_ns - received_ns <= timeout_ns：未超过该通道的超时窗口。
bool SafetyGate::fresh(
  const std::int64_t now_ns, const std::int64_t received_ns,
  const std::int64_t timeout_ns) const noexcept
{
  return received_ns > 0 && now_ns >= received_ns && now_ns - received_ns <= timeout_ns;
}

// 根据请求模式、输出开关、算法就绪标志和输入状态做一次完整裁决。
// 注意：本函数允许被节点在同一个 tick 内调用两次（reentry 双次评估）。
SafetyDecision SafetyGate::evaluate(
  const std::int64_t requested_mode, const bool enable_control_output,
  const bool core_algorithm_ready, const SafetyInputs & inputs) const noexcept
{
  // 模式越界直接返回：requested_mode / active_mode 保持默认的 Native，
  // 调用方通过 FaultInvalidMode 识别失败。
  SafetyDecision decision{};

  if (requested_mode < static_cast<std::int64_t>(Mode::Native) ||
    requested_mode > static_cast<std::int64_t>(Mode::FullControl))
  {
    decision.fault_flags |= FaultInvalidMode;
    return decision;
  }

  decision.requested_mode = static_cast<Mode>(requested_mode);
  // shadow_or_active：是否至少进入影子观测模式（1/2/3）。
  // active_requested：是否请求真正接管控制（2/3）。
  const bool shadow_or_active = decision.requested_mode != Mode::Native;
  const bool active_requested = decision.requested_mode == Mode::BoundedCompensation ||
    decision.requested_mode == Mode::FullControl;

  // ---------------------------------------------------------------------------
  // 第一层：基础状态通道。每个通道都要求「消息有效」且「接收未超时」。
  // valid 标志由节点根据消息自带的有效位和数值有限性计算；
  // 这里只负责把两者合成对应的故障位。
  // ---------------------------------------------------------------------------
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
  // 控制分配器状态使用单独的超时阈值（allocator_timeout_ns），因为它不是
  // 状态估计的一部分，而是 PX4 分配器的健康度反馈。
  if (!inputs.allocator_status_valid ||
    !fresh(inputs.now_ns, inputs.allocator_status_received_ns, config_.allocator_timeout_ns))
  {
    decision.fault_flags |= FaultAllocatorStatus;
  }

  // ---------------------------------------------------------------------------
  // 第二层：仅 Shadow 及以上模式需要的通道。
  // Native 模式只观测并透传 PX4 原生控制，不读取参考设定点，也不消费
  // 原生推力/力矩镜像，因此不把它们算作故障。
  // ---------------------------------------------------------------------------
  if (shadow_or_active &&
    (!inputs.reference_valid ||
    !fresh(inputs.now_ns, inputs.reference_received_ns, config_.reference_timeout_ns)))
  {
    decision.fault_flags |= FaultReference;
  }
  if (shadow_or_active &&
    (!inputs.native_thrust_valid ||
    !fresh(inputs.now_ns, inputs.native_thrust_received_ns, config_.native_control_timeout_ns)))
  {
    decision.fault_flags |= FaultNativeThrust;
  }
  if (shadow_or_active &&
    (!inputs.native_torque_valid ||
    !fresh(inputs.now_ns, inputs.native_torque_received_ns, config_.native_control_timeout_ns)))
  {
    decision.fault_flags |= FaultNativeTorque;
  }

  // ---------------------------------------------------------------------------
  // 第三层：与模式无关的全局健康检查。
  // 时间戳单调性、估计器复位、数值有限性、参数合法性、PX4 失效保护。
  // ---------------------------------------------------------------------------
  if (!inputs.timestamp_monotonic) {
    decision.fault_flags |= FaultTimestamp;
  }
  if (inputs.estimator_reset) {
    decision.fault_flags |= FaultEstimatorReset;
  }
  if (!inputs.values_finite) {
    decision.fault_flags |= FaultNonFinite;
  }
  if (!inputs.parameters_valid) {
    decision.fault_flags |= FaultParameterInvalid;
  }
  if (inputs.failsafe) {
    decision.fault_flags |= FaultFailsafe;
  }

  // ---------------------------------------------------------------------------
  // 第四层：主动模式（2/3）的起飞前提。
  // Shadow 模式允许未解锁、未 Offboard，因为它不会发布控制指令。
  // ---------------------------------------------------------------------------
  if (active_requested) {
    if (!inputs.armed) {
      decision.fault_flags |= FaultNotArmed;
    }
    if (!inputs.offboard) {
      decision.fault_flags |= FaultNotOffboard;
    }
    // 发生过故障/复位后，必须回到 Shadow 或 Native 再重新进入主动模式。
    if (inputs.reentry_required) {
      decision.fault_flags |= FaultReentryRequired;
    }
    // 占位参数默认禁止用于主动控制；仅当 SITL 显式放行时才继续检查
    // 仿真上下文（use_sim_time + 时间对齐），否则同样视为故障。
    if (inputs.has_placeholder && !inputs.placeholder_allowed) {
      decision.fault_flags |= FaultPlaceholderParameters;
    }
    if (inputs.has_placeholder && inputs.placeholder_allowed && !inputs.simulation_context_valid) {
      decision.fault_flags |= FaultSimulationContext;
    }
  }

  // 先按目前已收集的故障刷新输入健康度；主动模式下稍后会再次刷新。
  decision.inputs_valid = decision.fault_flags == FaultNone;

  // Native / Shadow 不发布控制指令：直接确认 active_mode 并返回。
  // 注意 Shadow 即使输入完全健康也不会置 publish_control。
  if (decision.requested_mode == Mode::Native || decision.requested_mode == Mode::Shadow) {
    decision.active_mode = decision.requested_mode;
    return decision;
  }

  // 主动模式还需要两个全局放行条件：核心算法就绪、输出总开关打开。
  // 这两项不依赖输入数据，因此放在主动模式分支的最后检查。
  if (!core_algorithm_ready) {
    decision.fault_flags |= FaultAlgorithmUnavailable;
  }
  if (!enable_control_output) {
    decision.fault_flags |= FaultOutputDisabled;
  }

  // 主动模式下 inputs_valid 的语义扩展为「输入健康 + 已放行控制」。
  decision.inputs_valid = decision.fault_flags == FaultNone;
  if (decision.inputs_valid) {
    decision.active_mode = decision.requested_mode;
    decision.publish_control = true;
  }
  return decision;
}

}  // namespace pregme
