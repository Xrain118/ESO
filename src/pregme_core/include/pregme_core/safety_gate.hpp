#pragma once

#include <cstdint>

#include "pregme_core/types.hpp"

// =============================================================================
// SafetyGate：无状态安全门接口
// -----------------------------------------------------------------------------
// 节点在固定的定时 tick 中把所有输入的健康度汇总成 SafetyInputs，交给
// SafetyGate::evaluate() 做纯函数式裁决，得到 SafetyDecision。
// 设计要点：
//   - 不含 I/O、不持有状态，方便单元测试和黄金向量复现；
//   - 故障位只累加不清除，任何故障都会阻止主动控制；
//   - 时间新鲜度用单调时钟（steady_clock）判断，不用 ROS/仿真时间。
// =============================================================================

namespace pregme
{

// 一次评估所需的完整输入快照。字段分三类：
//   1) *_received_ns：该通道最近一次被接受的接收时刻（单调时钟，纳秒），
//      0 表示从未收到；
//   2) *_valid：该通道数据当前是否可用（消息有效位 + 数值有限性）；
//   3) 全局标志：时间戳单调性、估计器复位、数值有限、解锁/Offboard/
//      失效保护、参数合法性、占位参数策略、仿真上下文、重入要求。
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
  // 全局健康与飞行前提。
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

// 各通道的接收超时阈值（纳秒）。默认值对应节点参数的 200/250 ms；
// 控制分配器单独使用 250 ms，其余状态/参考/原生控制通道共用 200 ms。
struct SafetyConfig {
  std::int64_t state_timeout_ns{200000000};
  std::int64_t allocator_timeout_ns{250000000};
  std::int64_t reference_timeout_ns{200000000};
  std::int64_t native_control_timeout_ns{200000000};
};

// 一次裁决的结果：
//   requested_mode  调用方请求的模式（非法模式下保持 Native）；
//   active_mode     实际生效的模式；
//   fault_flags     本次累计的故障位掩码；
//   inputs_valid    输入是否健康；主动模式下语义扩展为「输入健康 + 已放行」；
//   publish_control 是否允许节点向 PX4 写控制指令（唯一放行开关）。
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

  // 纯函数裁决：同样的参数和输入必然得到同样的结果，不使用任何成员可变状态。
  // 参数 enable_control_output()/core_algorithm_ready 是主动模式的两个硬门。
  SafetyDecision evaluate(
    std::int64_t requested_mode, bool enable_control_output,
    bool core_algorithm_ready, const SafetyInputs & inputs) const noexcept;

private:
  // 新鲜度判断：接收时刻非零、未超时，且当前时刻不早于接收时刻。
  bool fresh(std::int64_t now_ns, std::int64_t received_ns, std::int64_t timeout_ns) const noexcept;
  // 超时配置在构造时固定，运行期不变。
  SafetyConfig config_;
};

}  // namespace pregme
