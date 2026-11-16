#pragma once

#include <cstdint>
#include <string>

// =============================================================================
// RuntimeConfig：节点参数的内存镜像
// -----------------------------------------------------------------------------
// 字段与 PregmeControlNode::declare_parameters() 声明一一对应，启动时由
// read_config() 从 ROS 参数读出，再由 validate_runtime_config() 校验。
// 约定：
//   - 数值字段的默认值就是「安全默认」：模式 0、输出关闭、夹具值为 0.0；
//   - 0.0 与 "none" 是「未提供」的哨兵值，启用夹具时这些字段必须被替换，
//     并补齐 id/hash/watermark 三元组；
//   - 时间单位统一为毫秒（字段名带 _ms），频率为 Hz，物理量带单位后缀。
// =============================================================================

namespace pregme_control
{

struct RuntimeConfig {
  // 运行模式与安全时序（运行期仅 mode/enable_control_output 可修改）。
  std::int64_t mode{0};
  bool enable_control_output{false};
  double status_rate_hz{50.0};
  std::int64_t state_timeout_ms{200};
  std::int64_t allocator_timeout_ms{250};
  bool has_placeholder{false};
  bool allow_placeholder_in_sitl{false};
  std::int64_t simulation_clock_timeout_ms{200};
  std::int64_t simulation_time_tolerance_ms{100};

  // 夹具身份三元组：id、参数集规范化哈希、结果水印。
  std::string fixture_id{"none"};
  std::string parameter_set_sha256{"none"};
  std::string fixture_watermark{"none"};

  // 夹具物理参数（0.0 表示未提供，启用时必须是有限正数且相互自洽）。
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

  // PX4 输入话题契约。默认值即冻结的接口，运行期不可修改。
  std::string local_position_topic{"/fmu/out/vehicle_local_position"};
  std::string attitude_topic{"/fmu/out/vehicle_attitude"};
  std::string angular_velocity_topic{"/fmu/out/vehicle_angular_velocity"};
  std::string vehicle_status_topic{"/fmu/out/vehicle_status_v1"};
  std::string allocator_status_topic{"/fmu/out/control_allocator_status"};
  std::string reference_topic{"/fmu/out/vehicle_local_position_setpoint"};
  std::string native_thrust_topic{"/fmu/out/vehicle_thrust_setpoint"};
  std::string native_torque_topic{"/fmu/out/vehicle_torque_setpoint"};
};

// 校验结果：失败时 reason 给出可直接记录/抛出的具体原因。
struct ConfigValidation {
  bool valid{false};
  std::string reason{};
};

// 全量校验配置。实现见 runtime_config.cpp；任何失败都会让节点拒绝启动。
ConfigValidation validate_runtime_config(const RuntimeConfig & config);

// 判断参数是否允许运行期热更新。目前只有 mode 与 enable_control_output。
bool is_runtime_mutable_parameter(const std::string & name) noexcept;

}  // namespace pregme_control
