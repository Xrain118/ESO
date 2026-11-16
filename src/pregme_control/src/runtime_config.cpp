#include "pregme_control/runtime_config.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>

// =============================================================================
// RuntimeConfig 启动期校验
// -----------------------------------------------------------------------------
// 角色：节点构造时对参数做一次完整合法性检查，任何不合法都返回失败原因，
//       由调用方（PregmeControlNode 构造函数）抛异常拒绝启动。
// 结构：
//   1. 与夹具无关的安全参数：模式范围、发布频率、各类超时、话题名；
//   2. 夹具分支：默认安全集（onboard_default）允许全部为默认/空值；
//      一旦出现任何夹具数值或身份标识，就要求身份三元组（id/hash/watermark）
//      齐全且相互匹配，并校验质量/惯量/采样周期的物理与逻辑自洽性。
// 目的：防止「半套参数集」被加载——要么完全不启用夹具，要么提供一套可
//       审计、可水印、内部一致的参数。
// =============================================================================

namespace pregme_control
{
namespace
{

// ROS 话题名必须是绝对名：以 '/' 开头、长度大于 1，且不含空格。
// 这里不接受相对名或重映射用的非法字符串，避免订阅悄悄落到错误的主题上。
bool absolute_topic(const std::string & topic)
{
  return topic.size() > 1U && topic.front() == '/' && topic.find(' ') == std::string::npos;
}

// 参数集哈希必须是 64 位小写十六进制字符串，与 config/*.yaml 的规范哈希
// 约定一致；大写或长度不符都视为无效，避免水印校验出现歧义。
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
  // ---------------------------------------------------------------------------
  // 第一组：运行模式与安全时序参数。
  // 所有边界都取闭区间；超时下限 20 ms 用于排除明显不合理的配置，
  // 上限 1000 ms 防止输入失效检测被拖得失去意义。
  // ---------------------------------------------------------------------------
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

  // ---------------------------------------------------------------------------
  // 第二组：PX4 话题契约。八个输入话题全部要求是绝对名，防止空字符串或
  // 带空格的参数导致订阅失败而不自知。
  // ---------------------------------------------------------------------------
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

  // ---------------------------------------------------------------------------
  // 第三组：夹具参数集分支。
  // 参数文件用 0.0 / "none" 表示「未提供」。只要任何夹具数值或身份标识
  // 偏离默认值，就认为用户意图启用参数集，从而进入严格校验。
  // ---------------------------------------------------------------------------
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
    // 3.1 身份三元组：必须能被水印唯一重建为 FIXTURE:<id>:<sha256 前 12 位>。
    //     这保证实验输出可以直接追溯回参数文件和元数据文件。
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

    // 3.2 质量与几何：机体质量必须为正，机械臂质量允许为 0（例如不带臂），
    //     轴距必须为正。非有限值（NaN/Inf）一律拒绝。
    if (!std::isfinite(config.body_mass_kg) || config.body_mass_kg <= 0.0 ||
      !std::isfinite(config.arm_mass_kg) || config.arm_mass_kg < 0.0 ||
      !std::isfinite(config.wheelbase_m) || config.wheelbase_m <= 0.0)
    {
      return {false, "fixture mass and geometry must be finite and positive"};
    }

    // 3.3 惯量与质量的物理自洽性。
    //     回转半径 r = sqrt(I/m) 把质量和对角惯量绑定在一起：只改其中一个
    //     而不同时更新另一个，会在这里被拦截。[0.1, 0.4] m 对应小型四旋翼
    //     平台的合理量级；该检查与 config_contract.py 的夹具校验保持一致。
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

    // 3.4 采样周期一致性：min <= max，且位置/姿态两个期望周期都必须落在
    //     [min, max] 窗口内，避免把上下界写反或让期望周期游离于窗口之外。
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

    // 3.5 期望姿态导数低通截止频率必须为有限正数；0 或负值会让滤波器失效。
    if (!std::isfinite(config.desired_attitude_derivative_cutoff_hz) ||
      config.desired_attitude_derivative_cutoff_hz <= 0.0)
    {
      return {false, "fixture derivative cutoff must be finite and positive"};
    }
  }
  return {true, "ok"};
}

// 运行期允许通过 ROS 参数服务修改的参数白名单：只有模式切换和输出总开关。
// 其余参数（超时、话题、夹具数值等）都是启动期参数，节点会拒绝运行期修改，
// 以免安全边界在飞行过程中被悄悄改写。
bool is_runtime_mutable_parameter(const std::string & name) noexcept
{
  return name == "mode" || name == "enable_control_output";
}

}  // namespace pregme_control
