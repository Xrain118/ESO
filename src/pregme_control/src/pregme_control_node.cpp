// =============================================================================
// pregme_control_node：PreGME 机载控制节点
// -----------------------------------------------------------------------------
// 职责：
//   1. 订阅 PX4 经 uXRCE-DDS 暴露的状态/参考/原生控制量（八个输入话题）；
//   2. 在每个 tick 组装 SafetyInputs，交给 pregme::SafetyGate 裁决；
//   3. 发布 ESO 自身的状态（PregmeStatus）与安全事件（SafetyEvent）；
//   4. 仅在安全门明确放行时，向 PX4 写 OffboardControlMode、总推力和三轴
//      力矩（三个输出话题），绝不直接控制电机。
//
// 模式（见 pregme_core/types.hpp）：
//   0=Native 纯 PX4 原生，只观测；1=Shadow 影子对比，不写控制；
//   2=BoundedCompensation、3=FullControl 才会发布控制，当前核心算法未就绪，
//   因此这两个模式在运行期必然被 SafetyGate 拦截。
// =============================================================================

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <eso_interfaces/msg/pregme_status.hpp>
#include <eso_interfaces/msg/safety_event.hpp>
#include <pregme_control/runtime_config.hpp>
#include <pregme_core/safety_gate.hpp>
#include <pregme_core/simulation_clock_guard.hpp>
#include <pregme_core/timestamp_tracker.hpp>
#include <pregme_core/types.hpp>
#include <px4_msgs/msg/control_allocator_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

namespace pregme_control
{

class PregmeControlNode final : public rclcpp::Node
{
public:
  PregmeControlNode()
  : Node("pregme_control")
  {
    // 启动顺序刻意固定：先声明并校验参数，任何不合法配置都在建立订阅/
    // 发布器之前就抛异常退出，避免节点带着错误参数进入运行状态。
    declare_parameters();
    config_ = read_config();
    const auto validation = validate_runtime_config(config_);
    if (!validation.valid) {
      throw std::invalid_argument("invalid PreGME configuration: " + validation.reason);
    }

    // 请求模式与输出开关用原子变量保存：timed tick 线程读取，参数回调线程
    // 写入，两者之间不加锁。
    requested_mode_.store(config_.mode);
    enable_control_output_.store(config_.enable_control_output);
    use_sim_time_ = get_parameter("use_sim_time").as_bool();
    // 预设 reason 字符串容量，避免周期性发布时反复分配内存。
    fault_reason_.reserve(384);
    status_message_.reason.reserve(384);
    event_message_.reason.reserve(384);
    // 夹具身份写入状态消息，使每条状态都可追溯到具体参数集与水印。
    status_message_.fixture_id = config_.fixture_id;
    status_message_.parameter_set_sha256 = config_.parameter_set_sha256;
    status_message_.fixture_watermark = config_.fixture_watermark;
    // 安全门配置：基础状态/参考/原生控制通道共用 state_timeout；
    // 只有控制分配器状态使用独立的 allocator_timeout。毫秒在这里转成纳秒。
    gate_ = pregme::SafetyGate(pregme::SafetyConfig{
        config_.state_timeout_ms * 1000000LL,
        config_.allocator_timeout_ms * 1000000LL,
        config_.state_timeout_ms * 1000000LL,
        config_.state_timeout_ms * 1000000LL});

    // 参数回调负责运行期热更新（只允许 mode 与 enable_control_output）。
    // 必须在创建订阅之前安装，确保任何后续参数变更都经过安全校验。
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&PregmeControlNode::on_parameters, this, std::placeholders::_1));

    // 输入话题统一使用 SensorDataQoS（best-effort、小队列）：与 PX4 的
    // uXRCE-DDS 发布习惯一致，也避免过期数据在队列中堆积。
    const auto sensor_qos = rclcpp::SensorDataQoS();
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      config_.local_position_topic, sensor_qos,
      std::bind(&PregmeControlNode::on_local_position, this, std::placeholders::_1));
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      config_.attitude_topic, sensor_qos,
      std::bind(&PregmeControlNode::on_attitude, this, std::placeholders::_1));
    angular_velocity_sub_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
      config_.angular_velocity_topic, sensor_qos,
      std::bind(&PregmeControlNode::on_angular_velocity, this, std::placeholders::_1));
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      config_.vehicle_status_topic, sensor_qos,
      std::bind(&PregmeControlNode::on_vehicle_status, this, std::placeholders::_1));
    allocator_status_sub_ = create_subscription<px4_msgs::msg::ControlAllocatorStatus>(
      config_.allocator_status_topic, sensor_qos,
      std::bind(&PregmeControlNode::on_allocator_status, this, std::placeholders::_1));
    reference_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPositionSetpoint>(
      config_.reference_topic, sensor_qos,
      std::bind(&PregmeControlNode::on_reference, this, std::placeholders::_1));
    native_thrust_sub_ = create_subscription<px4_msgs::msg::VehicleThrustSetpoint>(
      config_.native_thrust_topic, sensor_qos,
      std::bind(&PregmeControlNode::on_native_thrust, this, std::placeholders::_1));
    native_torque_sub_ = create_subscription<px4_msgs::msg::VehicleTorqueSetpoint>(
      config_.native_torque_topic, sensor_qos,
      std::bind(&PregmeControlNode::on_native_torque, this, std::placeholders::_1));

    // ESO 自身诊断输出：状态按固定频率发布，事件只在边界变化时发布。
    status_pub_ = create_publisher<eso_interfaces::msg::PregmeStatus>("/eso/pregme/status", 10);
    event_pub_ = create_publisher<eso_interfaces::msg::SafetyEvent>(
      "/eso/pregme/safety_event", 10);

    // ESO 只拥有以下三个飞行控制发布器。公式核心通过黄金向量前，双重门控禁止发布。
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    thrust_pub_ = create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
      "/fmu/in/vehicle_thrust_setpoint", 10);
    torque_pub_ = create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
      "/fmu/in/vehicle_torque_setpoint", 10);

    // 主循环：周期由 status_rate_hz 决定（默认 50 Hz），同时驱动安全评估、
    // 状态发布和事件检测。
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / config_.status_rate_hz)),
      std::bind(&PregmeControlNode::tick, this));

    // 明确告警：公式已冻结，但可执行核心尚未实现，模式 2/3 实际上不可用。
    RCLCPP_WARN(
      get_logger(),
      "PreGME formulas are frozen but the executable core is unavailable; modes 2 and 3 remain disabled");
  }

private:
  // 事件来源：与 SafetyEvent.msg 中的 SOURCE_* 常量一一对应，用于告诉地面
  // 或日志分析端「是哪一路输入触发的这次事件」。
  enum class EventSource : std::uint8_t {
    Gate = eso_interfaces::msg::SafetyEvent::SOURCE_GATE,
    LocalPosition = eso_interfaces::msg::SafetyEvent::SOURCE_LOCAL_POSITION,
    Attitude = eso_interfaces::msg::SafetyEvent::SOURCE_ATTITUDE,
    AngularVelocity = eso_interfaces::msg::SafetyEvent::SOURCE_ANGULAR_VELOCITY,
    VehicleStatus = eso_interfaces::msg::SafetyEvent::SOURCE_VEHICLE_STATUS,
    AllocatorStatus = eso_interfaces::msg::SafetyEvent::SOURCE_ALLOCATOR_STATUS,
    Reference = eso_interfaces::msg::SafetyEvent::SOURCE_REFERENCE,
    NativeThrust = eso_interfaces::msg::SafetyEvent::SOURCE_NATIVE_THRUST,
    NativeTorque = eso_interfaces::msg::SafetyEvent::SOURCE_NATIVE_TORQUE,
    Parameters = eso_interfaces::msg::SafetyEvent::SOURCE_PARAMETERS,
    SimulationClock = eso_interfaces::msg::SafetyEvent::SOURCE_SIMULATION_CLOCK,
  };

  // 单调时钟（steady_clock）读取当前时间，单位纳秒。
  // 超时判断必须基于单调时钟而不是 ROS/仿真时间：仿真时间可能暂停或跳变，
  // 用它做超时会误报或漏报。
  static std::int64_t steady_now_ns() noexcept
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  // 向量数值有限性检查：用于把 NaN/Inf 数据挡在状态机之外。
  static bool finite3(const std::array<float, 3> & values) noexcept
  {
    return std::all_of(values.begin(), values.end(), [](const float value) {
      return std::isfinite(value);
    });
  }

  static bool finite4(const std::array<float, 4> & values) noexcept
  {
    return std::all_of(values.begin(), values.end(), [](const float value) {
      return std::isfinite(value);
    });
  }

  // 参数声明：这里的默认值是「安全默认」——Native 模式、控制输出关闭、
  // 不携带任何夹具数值。真正的取值由参数文件（config/*.yaml）覆盖，
  // 覆盖后的完整配置再由 read_config() 读出并校验。
  void declare_parameters()
  {
    // 运行模式与安全时序参数。
    declare_parameter<int>("mode", 0);
    declare_parameter<bool>("enable_control_output", false);
    declare_parameter<double>("status_rate_hz", 50.0);
    declare_parameter<int>("state_timeout_ms", 200);
    declare_parameter<int>("allocator_timeout_ms", 250);
    declare_parameter<bool>("allow_placeholder_in_sitl", false);
    declare_parameter<int>("simulation_clock_timeout_ms", 200);
    declare_parameter<int>("simulation_time_tolerance_ms", 100);
    // 夹具身份：id + 参数集哈希 + 水印。默认 none 表示不使用夹具。
    declare_parameter<bool>("fixture.has_placeholder", false);
    declare_parameter<std::string>("fixture.id", "none");
    declare_parameter<std::string>("fixture.parameter_set_sha256", "none");
    declare_parameter<std::string>("fixture.watermark", "none");
    // 夹具物理参数：质量、几何、对角惯量、期望采样周期与导数滤波截止频率。
    // 0.0 是「未提供」的哨兵值，启用夹具时由 validate_runtime_config()
    // 强制要求这些值为有限正数并保持内部自洽。
    declare_parameter<double>("fixture.body_mass_kg", 0.0);
    declare_parameter<double>("fixture.arm_mass_kg", 0.0);
    declare_parameter<double>("fixture.wheelbase_m", 0.0);
    declare_parameter<double>("fixture.inertia_xx_kg_m2", 0.0);
    declare_parameter<double>("fixture.inertia_yy_kg_m2", 0.0);
    declare_parameter<double>("fixture.inertia_zz_kg_m2", 0.0);
    declare_parameter<double>("fixture.expected_position_period_s", 0.0);
    declare_parameter<double>("fixture.expected_attitude_period_s", 0.0);
    declare_parameter<double>("fixture.minimum_period_s", 0.0);
    declare_parameter<double>("fixture.maximum_period_s", 0.0);
    declare_parameter<double>("fixture.desired_attitude_derivative_cutoff_hz", 0.0);
    // PX4 话题契约：默认值即冻结的接口约定，不允许在运行期修改。
    declare_parameter<std::string>("topics.local_position", "/fmu/out/vehicle_local_position");
    declare_parameter<std::string>("topics.attitude", "/fmu/out/vehicle_attitude");
    declare_parameter<std::string>(
      "topics.angular_velocity", "/fmu/out/vehicle_angular_velocity");
    declare_parameter<std::string>("topics.vehicle_status", "/fmu/out/vehicle_status_v1");
    declare_parameter<std::string>(
      "topics.allocator_status", "/fmu/out/control_allocator_status");
    declare_parameter<std::string>(
      "topics.reference", "/fmu/out/vehicle_local_position_setpoint");
    declare_parameter<std::string>(
      "topics.native_thrust", "/fmu/out/vehicle_thrust_setpoint");
    declare_parameter<std::string>(
      "topics.native_torque", "/fmu/out/vehicle_torque_setpoint");
  }

  // 把 ROS 参数读取为扁平结构体。字段与 declare_parameters() 一一对应，
  // 这里不设默认值，因为默认值已在声明阶段确定。
  RuntimeConfig read_config() const
  {
    RuntimeConfig config{};
    config.mode = get_parameter("mode").as_int();
    config.enable_control_output = get_parameter("enable_control_output").as_bool();
    config.status_rate_hz = get_parameter("status_rate_hz").as_double();
    config.state_timeout_ms = get_parameter("state_timeout_ms").as_int();
    config.allocator_timeout_ms = get_parameter("allocator_timeout_ms").as_int();
    config.allow_placeholder_in_sitl = get_parameter("allow_placeholder_in_sitl").as_bool();
    config.simulation_clock_timeout_ms = get_parameter("simulation_clock_timeout_ms").as_int();
    config.simulation_time_tolerance_ms = get_parameter("simulation_time_tolerance_ms").as_int();
    config.has_placeholder = get_parameter("fixture.has_placeholder").as_bool();
    config.fixture_id = get_parameter("fixture.id").as_string();
    config.parameter_set_sha256 = get_parameter("fixture.parameter_set_sha256").as_string();
    config.fixture_watermark = get_parameter("fixture.watermark").as_string();
    config.body_mass_kg = get_parameter("fixture.body_mass_kg").as_double();
    config.arm_mass_kg = get_parameter("fixture.arm_mass_kg").as_double();
    config.wheelbase_m = get_parameter("fixture.wheelbase_m").as_double();
    config.inertia_xx_kg_m2 = get_parameter("fixture.inertia_xx_kg_m2").as_double();
    config.inertia_yy_kg_m2 = get_parameter("fixture.inertia_yy_kg_m2").as_double();
    config.inertia_zz_kg_m2 = get_parameter("fixture.inertia_zz_kg_m2").as_double();
    config.expected_position_period_s =
      get_parameter("fixture.expected_position_period_s").as_double();
    config.expected_attitude_period_s =
      get_parameter("fixture.expected_attitude_period_s").as_double();
    config.minimum_period_s = get_parameter("fixture.minimum_period_s").as_double();
    config.maximum_period_s = get_parameter("fixture.maximum_period_s").as_double();
    config.desired_attitude_derivative_cutoff_hz =
      get_parameter("fixture.desired_attitude_derivative_cutoff_hz").as_double();
    config.local_position_topic = get_parameter("topics.local_position").as_string();
    config.attitude_topic = get_parameter("topics.attitude").as_string();
    config.angular_velocity_topic = get_parameter("topics.angular_velocity").as_string();
    config.vehicle_status_topic = get_parameter("topics.vehicle_status").as_string();
    config.allocator_status_topic = get_parameter("topics.allocator_status").as_string();
    config.reference_topic = get_parameter("topics.reference").as_string();
    config.native_thrust_topic = get_parameter("topics.native_thrust").as_string();
    config.native_torque_topic = get_parameter("topics.native_torque").as_string();
    return config;
  }

  // 判断某个参数是否由本节点声明并管理。
  // 只有本节点拥有的参数才受「启动期参数不可运行期修改」规则约束；
  // 未拥有的参数直接跳过，交给其他回调处理。
  static bool owned_parameter(const std::string & name)
  {
    return name == "mode" || name == "enable_control_output" || name == "status_rate_hz" ||
           name == "state_timeout_ms" || name == "allocator_timeout_ms" ||
           name == "allow_placeholder_in_sitl" || name == "simulation_clock_timeout_ms" ||
           name == "simulation_time_tolerance_ms" || name == "use_sim_time" ||
           name.rfind("topics.", 0) == 0 || name.rfind("fixture.", 0) == 0;
  }

  // ROS 参数热更新回调。采用「先全部校验、再一次性提交」的两阶段策略：
  // 循环里只写局部变量 mode/enable，任何一项校验失败都直接返回失败，
  // 已经通过校验的参数也不会被部分应用。
  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result{};
    result.successful = false;
    auto mode = requested_mode_.load();
    auto enable = enable_control_output_.load();

    for (const auto & parameter : parameters) {
      // 任何启动期参数（超时、话题、夹具值等）都不允许运行期修改；
      // 返回失败会让整批参数变更一起被拒绝。
      if (owned_parameter(parameter.get_name()) &&
        !is_runtime_mutable_parameter(parameter.get_name()))
      {
        result.reason = parameter.get_name() + " is startup-only; restart the node to change it";
        return result;
      }
      // 模式切换：类型和取值必须同时合法，避免字符串或越界值被写入。
      if (parameter.get_name() == "mode") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          result.reason = "mode must be an integer";
          return result;
        }
        mode = parameter.as_int();
        if (mode < 0 || mode > 3) {
          result.reason = "mode must be in [0, 3]";
          return result;
        }
      } else if (parameter.get_name() == "enable_control_output") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.reason = "enable_control_output must be boolean";
          return result;
        }
        enable = parameter.as_bool();
      }
    }

    // 全部校验通过后一次性提交到原子变量。
    requested_mode_.store(mode);
    enable_control_output_.store(enable);
    // 回到 Native/Shadow 视为退出主动控制：下一个 tick 清除 reentry 锁存，
    // 允许之后重新进入模式 2/3（这是解除重入故障的唯一途径）。
    if (mode <= static_cast<std::int64_t>(pregme::Mode::Shadow)) {
      reentry_clear_requested_ = true;
    }
    result.successful = true;
    result.reason = "ok";
    return result;
  }

  // 所有消息共用的时间戳闸门：
  //   - InvalidZero / Regression：异常时间戳，锁存 reentry 并登记事件来源，
  //     本帧消息被丢弃；
  //   - First / Advanced：新数据，返回 true，允许后续处理；
  //   - Duplicate：重复时间戳，返回 false，不刷新状态（避免陈旧数据续命）。
  bool accept_timestamp(
    pregme::TimestampTracker & tracker, const std::uint64_t timestamp_us,
    const EventSource source)
  {
    const auto result = tracker.observe(timestamp_us);
    if (result == pregme::TimestampResult::InvalidZero ||
      result == pregme::TimestampResult::Regression)
    {
      pending_timestamp_event_ = true;
      pending_event_source_ = source;
      reentry_required_ = true;
      return false;
    }
    return pregme::timestamp_is_new(result);
  }

  // 任一路通道锁存了时间戳故障即为真。用于组装 SafetyInputs.timestamp_monotonic。
  bool any_timestamp_fault() const noexcept
  {
    return local_timestamp_.fault_latched() || attitude_timestamp_.fault_latched() ||
           angular_velocity_timestamp_.fault_latched() || vehicle_status_timestamp_.fault_latched() ||
           allocator_timestamp_.fault_latched() || reference_timestamp_.fault_latched() ||
           native_thrust_timestamp_.fault_latched() || native_torque_timestamp_.fault_latched();
  }

  // PX4 时间戳整体回退意味着飞控重启、SITL 重开或时间基准切换，不是普通
  // 的单通道抖动。此时不能只报一次故障，而要把整个输入世界清空：
  //   - 重置所有时间戳跟踪器和仿真时钟守卫；
  //   - 作废所有通道的 valid 标志与接收时刻；
  //   - 清除解锁/Offboard/失效保护和估计器复位计数；
  //   - 以本帧 vehicle_status 时间重新播种时间基准，并锁存重入要求。
  // 这样后续消息会以 First/Advanced 重新建立基线，而不是被误判为回退。
  void start_new_px4_time_epoch(const std::uint64_t vehicle_status_timestamp_us)
  {
    local_timestamp_.reset();
    attitude_timestamp_.reset();
    angular_velocity_timestamp_.reset();
    vehicle_status_timestamp_.reset();
    allocator_timestamp_.reset();
    reference_timestamp_.reset();
    native_thrust_timestamp_.reset();
    native_torque_timestamp_.reset();
    simulation_clock_guard_.reset();

    local_valid_ = false;
    attitude_valid_ = false;
    angular_velocity_valid_ = false;
    allocator_status_valid_ = false;
    reference_valid_ = false;
    native_thrust_valid_ = false;
    native_torque_valid_ = false;
    vehicle_status_valid_ = false;
    local_received_ns_ = 0;
    attitude_received_ns_ = 0;
    angular_velocity_received_ns_ = 0;
    vehicle_status_received_ns_ = 0;
    allocator_status_received_ns_ = 0;
    reference_received_ns_ = 0;
    native_thrust_received_ns_ = 0;
    native_torque_received_ns_ = 0;
    armed_ = false;
    offboard_ = false;
    failsafe_ = false;
    local_reset_initialized_ = false;
    attitude_reset_initialized_ = false;
    local_reset_counters_ = {};
    attitude_reset_counter_ = 0;

    // 用当前帧重新建立该通道基线，并登记一次 ResetPx4TimeEpoch 事件。
    vehicle_status_timestamp_.observe(vehicle_status_timestamp_us);
    pending_timestamp_event_ = true;
    pending_event_source_ = EventSource::VehicleStatus;
    pending_reset_reason_flags_ |= pregme::ResetPx4TimeEpoch;
    last_reset_reason_flags_ = pending_reset_reason_flags_;
    ++reset_count_;
    reentry_required_ = true;
  }

  // 局部位置/速度（NED）。valid 要求 xy/z 位置和 xy/z 速度四个有效位同时
  // 为真，并且数值有限。
  void on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr message)
  {
    if (!accept_timestamp(
        local_timestamp_, message->timestamp_sample, EventSource::LocalPosition))
    {
      return;
    }
    local_received_ns_ = steady_now_ns();
    state_.local_position_timestamp_sample_us = message->timestamp_sample;
    state_.position_ned = {message->x, message->y, message->z};
    state_.velocity_ned = {message->vx, message->vy, message->vz};
    local_finite_ = finite3(state_.position_ned) && finite3(state_.velocity_ned);
    local_valid_ = message->xy_valid && message->z_valid && message->v_xy_valid &&
      message->v_z_valid && local_finite_;

    // 估计器复位检测：PX4 在 EKF 重置位置/速度时会递增对应的 reset_counter。
    // 首帧只记录基线（本地坐标可能与上次会话无关）；之后计数器变化即视为
    // 一次估计器复位，记录原因位并要求重新进入主动模式。
    std::uint32_t reset_reasons = pregme::ResetNone;
    if (local_reset_initialized_) {
      if (message->xy_reset_counter != local_reset_counters_[0]) {
        reset_reasons |= pregme::ResetPositionXY;
      }
      if (message->z_reset_counter != local_reset_counters_[1]) {
        reset_reasons |= pregme::ResetPositionZ;
      }
      if (message->vxy_reset_counter != local_reset_counters_[2]) {
        reset_reasons |= pregme::ResetVelocityXY;
      }
      if (message->vz_reset_counter != local_reset_counters_[3]) {
        reset_reasons |= pregme::ResetVelocityZ;
      }
    }
    local_reset_counters_ = {
      message->xy_reset_counter, message->z_reset_counter,
      message->vxy_reset_counter, message->vz_reset_counter};
    local_reset_initialized_ = true;
    if (reset_reasons != pregme::ResetNone) {
      pending_reset_reason_flags_ |= reset_reasons;
      last_reset_reason_flags_ = reset_reasons;
      ++reset_count_;
      pending_event_source_ = EventSource::LocalPosition;
      reentry_required_ = true;
    }
  }

  // 姿态四元数（w,x,y,z，机体系到 NED）。除有限性外还检查范数平方落在
  // [0.81, 1.21]，即 |q| 约在 [0.9, 1.1]，容忍浮点/滤波误差的同时拒绝
  // 明显未归一化或损坏的四元数。
  void on_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr message)
  {
    if (!accept_timestamp(attitude_timestamp_, message->timestamp_sample, EventSource::Attitude)) {
      return;
    }
    attitude_received_ns_ = steady_now_ns();
    state_.attitude_timestamp_sample_us = message->timestamp_sample;
    std::copy(message->q.begin(), message->q.end(), state_.attitude_wxyz.begin());
    const float norm_squared =
      state_.attitude_wxyz[0] * state_.attitude_wxyz[0] +
      state_.attitude_wxyz[1] * state_.attitude_wxyz[1] +
      state_.attitude_wxyz[2] * state_.attitude_wxyz[2] +
      state_.attitude_wxyz[3] * state_.attitude_wxyz[3];
    attitude_finite_ = finite4(state_.attitude_wxyz);
    attitude_valid_ = attitude_finite_ && norm_squared >= 0.81F && norm_squared <= 1.21F;

    // 与位置通道同理：quat_reset_counter 变化表示姿态估计被重置。
    if (attitude_reset_initialized_ &&
      message->quat_reset_counter != attitude_reset_counter_)
    {
      pending_reset_reason_flags_ |= pregme::ResetAttitude;
      last_reset_reason_flags_ = pregme::ResetAttitude;
      ++reset_count_;
      pending_event_source_ = EventSource::Attitude;
      reentry_required_ = true;
    }
    attitude_reset_counter_ = message->quat_reset_counter;
    attitude_reset_initialized_ = true;
  }

  // 机体角速度（FRD）。仅要求数值有限；valid 直接跟随有限性检查。
  void on_angular_velocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr message)
  {
    if (!accept_timestamp(
        angular_velocity_timestamp_, message->timestamp_sample, EventSource::AngularVelocity))
    {
      return;
    }
    angular_velocity_received_ns_ = steady_now_ns();
    state_.angular_velocity_timestamp_sample_us = message->timestamp_sample;
    std::copy(message->xyz.begin(), message->xyz.end(), state_.angular_velocity_frd.begin());
    angular_velocity_finite_ = finite3(state_.angular_velocity_frd);
    angular_velocity_valid_ = angular_velocity_finite_;
  }

  // 飞行器状态（解锁/导航模式/失效保护），同时也是 PX4 时间基准的锚点：
  // 该通道的时间戳回退被解释为「PX4 时间纪元切换」，会触发全量重置。
  void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr message)
  {
    const auto timestamp_result = vehicle_status_timestamp_.observe(message->timestamp);
    if (timestamp_result == pregme::TimestampResult::Regression) {
      start_new_px4_time_epoch(message->timestamp);
    } else if (timestamp_result == pregme::TimestampResult::InvalidZero) {
      pending_timestamp_event_ = true;
      pending_event_source_ = EventSource::VehicleStatus;
      reentry_required_ = true;
      return;
    } else if (!pregme::timestamp_is_new(timestamp_result)) {
      return;
    }

    const auto received_ns = steady_now_ns();
    vehicle_status_received_ns_ = received_ns;
    state_.vehicle_status_timestamp_us = message->timestamp;
    vehicle_status_valid_ = true;
    // 主动模式的两项硬前提：已解锁且处于 Offboard；失效保护另行单列。
    armed_ = message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    offboard_ = message->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    failsafe_ = message->failsafe;
    // 仿真时钟守卫：比较 ROS 时间与 PX4 时间戳的偏移是否稳定在容差内，
    // 用来确认 SITL 的 use_sim_time 与 PX4 时钟确实对齐。
    simulation_clock_guard_.observe(
      get_clock()->now().nanoseconds(), message->timestamp, received_ns,
      config_.simulation_time_tolerance_ms * 1000000LL);
  }

  // PX4 控制分配器反馈。unallocated_* 表示分配器「没能实现」的推力/力矩
  // 残差；它只用于诊断与影子对比，不参与 ESO 控制。
  // valid 要求：残差有限、推力/力矩设定点均已达成、无电机故障被置位。
  void on_allocator_status(const px4_msgs::msg::ControlAllocatorStatus::SharedPtr message)
  {
    if (!accept_timestamp(allocator_timestamp_, message->timestamp, EventSource::AllocatorStatus)) {
      return;
    }
    allocator_status_received_ns_ = steady_now_ns();
    state_.allocator_timestamp_us = message->timestamp;
    std::array<float, 3> unallocated_thrust{};
    std::array<float, 3> unallocated_torque{};
    std::copy(
      message->unallocated_thrust.begin(), message->unallocated_thrust.end(),
      unallocated_thrust.begin());
    std::copy(
      message->unallocated_torque.begin(), message->unallocated_torque.end(),
      unallocated_torque.begin());
    allocator_finite_ = finite3(unallocated_thrust) && finite3(unallocated_torque);
    allocator_status_valid_ = allocator_finite_ && message->torque_setpoint_achieved &&
      message->thrust_setpoint_achieved && message->handled_motor_failure_mask == 0U;
  }

  // 位置参考设定点（PX4 原生位置控制器的输出）：位置/速度/加速度/yaw 及
  // yaw 角速度。Shadow 及以上模式用它与 ESO 期望轨迹做对比。
  void on_reference(const px4_msgs::msg::VehicleLocalPositionSetpoint::SharedPtr message)
  {
    if (!accept_timestamp(reference_timestamp_, message->timestamp, EventSource::Reference)) {
      return;
    }
    reference_received_ns_ = steady_now_ns();
    state_.reference_timestamp_us = message->timestamp;
    state_.reference_position_ned = {message->x, message->y, message->z};
    state_.reference_velocity_ned = {message->vx, message->vy, message->vz};
    std::copy(
      message->acceleration.begin(), message->acceleration.end(),
      state_.reference_acceleration_ned.begin());
    state_.reference_yaw_ned = message->yaw;
    state_.reference_yawspeed_ned = message->yawspeed;
    reference_finite_ = finite3(state_.reference_position_ned) &&
      finite3(state_.reference_velocity_ned) && finite3(state_.reference_acceleration_ned) &&
      std::isfinite(state_.reference_yaw_ned) && std::isfinite(state_.reference_yawspeed_ned);
    reference_valid_ = reference_finite_;
  }

  // PX4 原生推力设定点镜像（FRD）。Shadow 模式用它作为 ESO 补偿前的基准；
  // Native 模式不消费，因此对应故障只在 Shadow 及以上模式生效。
  void on_native_thrust(const px4_msgs::msg::VehicleThrustSetpoint::SharedPtr message)
  {
    if (!accept_timestamp(
        native_thrust_timestamp_, message->timestamp_sample, EventSource::NativeThrust))
    {
      return;
    }
    native_thrust_received_ns_ = steady_now_ns();
    state_.native_thrust_timestamp_sample_us = message->timestamp_sample;
    std::copy(message->xyz.begin(), message->xyz.end(), state_.native_thrust_setpoint_frd.begin());
    native_thrust_finite_ = finite3(state_.native_thrust_setpoint_frd);
    native_thrust_valid_ = native_thrust_finite_;
  }

  // PX4 原生力矩设定点镜像（FRD），用途同 on_native_thrust。
  void on_native_torque(const px4_msgs::msg::VehicleTorqueSetpoint::SharedPtr message)
  {
    if (!accept_timestamp(
        native_torque_timestamp_, message->timestamp_sample, EventSource::NativeTorque))
    {
      return;
    }
    native_torque_received_ns_ = steady_now_ns();
    state_.native_torque_timestamp_sample_us = message->timestamp_sample;
    std::copy(message->xyz.begin(), message->xyz.end(), state_.native_torque_setpoint_frd.begin());
    native_torque_finite_ = finite3(state_.native_torque_setpoint_frd);
    native_torque_valid_ = native_torque_finite_;
  }

  // 把各通道 valid 布尔值打包成 PregmeStatus.validity_flags 位掩码，
  // 便于订阅端一次读取全部通道健康度。
  std::uint32_t validity_flags() const noexcept
  {
    std::uint32_t flags = 0U;
    if (local_valid_) {flags |= eso_interfaces::msg::PregmeStatus::VALID_LOCAL_POSITION;}
    if (attitude_valid_) {flags |= eso_interfaces::msg::PregmeStatus::VALID_ATTITUDE;}
    if (angular_velocity_valid_) {
      flags |= eso_interfaces::msg::PregmeStatus::VALID_ANGULAR_VELOCITY;
    }
    if (vehicle_status_valid_) {flags |= eso_interfaces::msg::PregmeStatus::VALID_VEHICLE_STATUS;}
    if (allocator_status_valid_) {
      flags |= eso_interfaces::msg::PregmeStatus::VALID_ALLOCATOR_STATUS;
    }
    if (reference_valid_) {flags |= eso_interfaces::msg::PregmeStatus::VALID_REFERENCE;}
    if (native_thrust_valid_) {flags |= eso_interfaces::msg::PregmeStatus::VALID_NATIVE_THRUST;}
    if (native_torque_valid_) {flags |= eso_interfaces::msg::PregmeStatus::VALID_NATIVE_TORQUE;}
    return flags;
  }

  // 把故障位掩码翻译成逗号分隔的稳定英文标签，写入状态与事件的 reason 字段。
  // 标签保持英文是为了让日志/脚本按固定字符串匹配，不随本地化变化。
  static void describe_faults(const std::uint32_t faults, std::string & reason)
  {
    reason.clear();
    if (faults == pregme::FaultNone) {
      reason.append("ok");
      return;
    }
    // 惰性拼接：第一个标签不加逗号，后续标签前补逗号。
    const auto add = [&reason](const char * text) {
        if (!reason.empty()) {reason.push_back(',');}
        reason.append(text);
      };
    if (faults & pregme::FaultInvalidMode) {add("invalid_mode");}
    if (faults & pregme::FaultLocalPosition) {add("local_position");}
    if (faults & pregme::FaultAttitude) {add("attitude");}
    if (faults & pregme::FaultAngularVelocity) {add("angular_velocity");}
    if (faults & pregme::FaultVehicleStatus) {add("vehicle_status");}
    if (faults & pregme::FaultAllocatorStatus) {add("allocator_status");}
    if (faults & pregme::FaultTimestamp) {add("timestamp");}
    if (faults & pregme::FaultEstimatorReset) {add("estimator_reset");}
    if (faults & pregme::FaultNonFinite) {add("non_finite");}
    if (faults & pregme::FaultAlgorithmUnavailable) {add("algorithm_unavailable");}
    if (faults & pregme::FaultOutputDisabled) {add("output_disabled");}
    if (faults & pregme::FaultReference) {add("reference");}
    if (faults & pregme::FaultNativeThrust) {add("native_thrust");}
    if (faults & pregme::FaultNativeTorque) {add("native_torque");}
    if (faults & pregme::FaultParameterInvalid) {add("parameter_invalid");}
    if (faults & pregme::FaultPlaceholderParameters) {add("placeholder_parameters");}
    if (faults & pregme::FaultSimulationContext) {add("simulation_context");}
    if (faults & pregme::FaultNotArmed) {add("not_armed");}
    if (faults & pregme::FaultNotOffboard) {add("not_offboard");}
    if (faults & pregme::FaultFailsafe) {add("failsafe");}
    if (faults & pregme::FaultReentryRequired) {add("reentry_required");}
  }

  // 周期主循环（默认 50 Hz），每拍执行：
  //   1. 处理重入解除请求；
  //   2. 检查仿真上下文；
  //   3. 汇总输入快照，调用 SafetyGate 裁决（必要时二次评估）；
  //   4. 发布状态、边沿事件，以及（若放行）控制指令。
  void tick()
  {
    const auto requested_mode = requested_mode_.load();
    // 重入锁存只在回到 Native/Shadow 后清除；清一次即消费掉请求。
    if (reentry_clear_requested_ && requested_mode <= static_cast<std::int64_t>(pregme::Mode::Shadow)) {
      reentry_required_ = false;
      reentry_clear_requested_ = false;
    }

    // Shadow 及以上模式会消费参考/原生控制镜像，因此 values_finite 的检查
    // 范围随模式变化；Native 模式不检查这两路。
    const bool shadow_or_active = requested_mode >= static_cast<std::int64_t>(pregme::Mode::Shadow);
    const auto now_ns = steady_now_ns();
    // 仿真上下文 = use_sim_time && 显式放行占位参数 && 仿真时钟近期对齐。
    // 它是占位参数进入主动控制的最后一道环境条件。
    const bool simulation_context_valid = simulation_clock_guard_.valid(
      now_ns, config_.simulation_clock_timeout_ms * 1000000LL,
      use_sim_time_, config_.allow_placeholder_in_sitl);

    // 汇总本拍的完整输入快照交给安全门；安全门本身不访问节点状态。
    pregme::SafetyInputs inputs{};
    inputs.now_ns = now_ns;
    inputs.local_position_received_ns = local_received_ns_;
    inputs.attitude_received_ns = attitude_received_ns_;
    inputs.angular_velocity_received_ns = angular_velocity_received_ns_;
    inputs.vehicle_status_received_ns = vehicle_status_received_ns_;
    inputs.allocator_status_received_ns = allocator_status_received_ns_;
    inputs.reference_received_ns = reference_received_ns_;
    inputs.native_thrust_received_ns = native_thrust_received_ns_;
    inputs.native_torque_received_ns = native_torque_received_ns_;
    inputs.local_position_valid = local_valid_;
    inputs.attitude_valid = attitude_valid_;
    inputs.angular_velocity_valid = angular_velocity_valid_;
    inputs.vehicle_status_valid = vehicle_status_valid_;
    inputs.allocator_status_valid = allocator_status_valid_;
    inputs.reference_valid = reference_valid_;
    inputs.native_thrust_valid = native_thrust_valid_;
    inputs.native_torque_valid = native_torque_valid_;
    inputs.timestamp_monotonic = !pending_timestamp_event_ && !any_timestamp_fault();
    inputs.estimator_reset = pending_reset_reason_flags_ != pregme::ResetNone;
    inputs.values_finite = local_finite_ && attitude_finite_ && angular_velocity_finite_ &&
      allocator_finite_ && (!shadow_or_active ||
      (reference_finite_ && native_thrust_finite_ && native_torque_finite_));
    inputs.armed = armed_;
    inputs.offboard = offboard_;
    inputs.failsafe = failsafe_;
    inputs.parameters_valid = true;
    inputs.has_placeholder = config_.has_placeholder;
    inputs.placeholder_allowed = config_.allow_placeholder_in_sitl;
    inputs.simulation_context_valid = simulation_context_valid;
    inputs.reentry_required = reentry_required_;

    // 第一次评估。若请求的是主动模式且出现了「非重入类」故障（数据类故障、
    // 未解锁/未 Offboard/失效保护等），需要把 reentry 锁存起来再评估一次，
    // 使本拍的状态就反映 FaultReentryRequired，而不是等到下一拍。
    auto decision = gate_.evaluate(
      requested_mode, enable_control_output_.load(), pregme::algorithm_ready(), inputs);
    // 这五个故障不触发 reentry：算法未实现、输出开关关闭、占位参数策略、
    // 仿真上下文不符、以及 reentry 本身。否则会形成自我锁死的循环。
    constexpr std::uint32_t non_reentry_faults =
      pregme::FaultAlgorithmUnavailable | pregme::FaultOutputDisabled |
      pregme::FaultPlaceholderParameters | pregme::FaultSimulationContext |
      pregme::FaultReentryRequired;
    if (requested_mode >= static_cast<std::int64_t>(pregme::Mode::BoundedCompensation) &&
      (decision.fault_flags & ~non_reentry_faults) != pregme::FaultNone)
    {
      reentry_required_ = true;
      inputs.reentry_required = true;
      decision = gate_.evaluate(
        requested_mode, enable_control_output_.load(), pregme::algorithm_ready(), inputs);
    }

    // 状态每拍发布；事件只在「离散事件 / 故障位变化 / 活动模式变化」时发布，
    // 避免高频刷屏。事件来源优先指向触发它的输入通道，否则归为 Gate。
    describe_faults(decision.fault_flags, fault_reason_);
    publish_status(decision, simulation_context_valid);
    const bool discrete_event = pending_timestamp_event_ ||
      pending_reset_reason_flags_ != pregme::ResetNone;
    if (discrete_event || decision.fault_flags != previous_fault_flags_ ||
      decision.active_mode != previous_active_mode_)
    {
      publish_event(decision, discrete_event ? pending_event_source_ : EventSource::Gate);
      previous_fault_flags_ = decision.fault_flags;
      previous_active_mode_ = decision.active_mode;
    }

    // 一次性事件标志在发布后清除；复位原因保留在 last_reset_reason_flags_ 里
    // 供后续状态/事件回看。
    pending_timestamp_event_ = false;
    pending_reset_reason_flags_ = pregme::ResetNone;

    // 当前控制输出为空且 valid=false（算法核心尚未实现），因此即使安全门
    // 放行也不会真的写入 PX4；接入算法后这里才会产生有效指令。
    const pregme::ControlOutput output{};
    if (decision.publish_control && output.valid) {
      publish_control(output);
    }
  }

  // 填充并发布 PreGmeStatus。复用成员消息对象以避免每拍重新分配；
  // 所有字段每拍都会被覆盖，不存在跨拍残留。
  void publish_status(
    const pregme::SafetyDecision & decision, const bool simulation_context_valid)
  {
    auto & message = status_message_;
    message.timestamp = static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
    message.timestamp_sample = state_.local_position_timestamp_sample_us;
    message.local_position_timestamp_sample = state_.local_position_timestamp_sample_us;
    message.attitude_timestamp_sample = state_.attitude_timestamp_sample_us;
    message.angular_velocity_timestamp_sample = state_.angular_velocity_timestamp_sample_us;
    message.vehicle_status_timestamp = state_.vehicle_status_timestamp_us;
    message.allocator_timestamp = state_.allocator_timestamp_us;
    message.reference_timestamp = state_.reference_timestamp_us;
    message.native_thrust_timestamp_sample = state_.native_thrust_timestamp_sample_us;
    message.native_torque_timestamp_sample = state_.native_torque_timestamp_sample_us;
    message.requested_mode = static_cast<std::uint8_t>(decision.requested_mode);
    message.active_mode = static_cast<std::uint8_t>(decision.active_mode);
    message.algorithm_ready = pregme::algorithm_ready();
    message.input_valid = decision.inputs_valid;
    message.output_enabled = decision.publish_control;
    message.has_placeholder = config_.has_placeholder;
    message.simulation_context_valid = simulation_context_valid;
    message.validity_flags = validity_flags();
    message.fault_flags = decision.fault_flags;
    message.reset_count = reset_count_;
    message.reset_reason_flags = last_reset_reason_flags_;
    message.reason = fault_reason_;
    message.position_ned = state_.position_ned;
    message.velocity_ned = state_.velocity_ned;
    message.attitude_wxyz = state_.attitude_wxyz;
    message.angular_velocity_frd = state_.angular_velocity_frd;
    message.reference_position_ned = state_.reference_position_ned;
    message.reference_velocity_ned = state_.reference_velocity_ned;
    message.reference_acceleration_ned = state_.reference_acceleration_ned;
    message.reference_yaw_ned = state_.reference_yaw_ned;
    message.reference_yawspeed_ned = state_.reference_yawspeed_ned;
    message.native_thrust_setpoint_frd = state_.native_thrust_setpoint_frd;
    message.native_torque_setpoint_frd = state_.native_torque_setpoint_frd;
    message.thrust_setpoint_frd = last_output_.thrust_frd;
    message.torque_setpoint_frd = last_output_.torque_frd;
    status_pub_->publish(message);
  }

  // 按事件来源取对应的采样时间戳，写入 SafetyEvent.timestamp_sample。
  // 未知来源（如 Gate/Parameters/SimulationClock）退化到局部位置时间戳。
  std::uint64_t event_timestamp_sample(const EventSource source) const noexcept
  {
    switch (source) {
      case EventSource::LocalPosition: return state_.local_position_timestamp_sample_us;
      case EventSource::Attitude: return state_.attitude_timestamp_sample_us;
      case EventSource::AngularVelocity: return state_.angular_velocity_timestamp_sample_us;
      case EventSource::VehicleStatus: return state_.vehicle_status_timestamp_us;
      case EventSource::AllocatorStatus: return state_.allocator_timestamp_us;
      case EventSource::Reference: return state_.reference_timestamp_us;
      case EventSource::NativeThrust: return state_.native_thrust_timestamp_sample_us;
      case EventSource::NativeTorque: return state_.native_torque_timestamp_sample_us;
      default: return state_.local_position_timestamp_sample_us;
    }
  }

  // 发布边沿安全事件。无故障为 INFO，有故障为 WARNING；event_sequence_
  // 单调递增，接收端可据此检测丢事件。
  void publish_event(const pregme::SafetyDecision & decision, const EventSource source)
  {
    auto & message = event_message_;
    message.timestamp = static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
    message.timestamp_sample = event_timestamp_sample(source);
    message.level = decision.fault_flags == pregme::FaultNone ?
      eso_interfaces::msg::SafetyEvent::LEVEL_INFO :
      eso_interfaces::msg::SafetyEvent::LEVEL_WARNING;
    message.source = static_cast<std::uint8_t>(source);
    message.requested_mode = static_cast<std::uint8_t>(decision.requested_mode);
    message.active_mode = static_cast<std::uint8_t>(decision.active_mode);
    message.fault_flags = decision.fault_flags;
    message.reset_reason_flags = pending_reset_reason_flags_;
    message.reset_count = reset_count_;
    message.event_sequence = ++event_sequence_;
    message.reason = fault_reason_;
    event_pub_->publish(message);
  }

  // 按 PX4 Offboard 契约发布控制量：先声明「使用推力+力矩」的
  // OffboardControlMode，再发推力与力矩设定点，三者时间戳同源。
  // timestamp_sample 分别标注所依据的状态采样时刻（推力用位置通道、
  // 力矩用姿态通道），方便下游评估延迟。
  void publish_control(const pregme::ControlOutput & output)
  {
    const auto timestamp = static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
    px4_msgs::msg::OffboardControlMode offboard{};
    offboard.timestamp = timestamp;
    offboard.thrust_and_torque = true;
    offboard_mode_pub_->publish(offboard);

    px4_msgs::msg::VehicleThrustSetpoint thrust{};
    thrust.timestamp = timestamp;
    thrust.timestamp_sample = state_.local_position_timestamp_sample_us;
    thrust.xyz = output.thrust_frd;
    thrust_pub_->publish(thrust);

    px4_msgs::msg::VehicleTorqueSetpoint torque{};
    torque.timestamp = timestamp;
    torque.timestamp_sample = state_.attitude_timestamp_sample_us;
    torque.xyz = output.torque_frd;
    torque_pub_->publish(torque);
    last_output_ = output;
  }

  // ---- 运行期核心对象 ----
  RuntimeConfig config_{};                 // 启动期校验后的完整配置
  pregme::SafetyGate gate_{};              // 无状态安全门
  pregme::InputState state_{};             // 最近一帧各通道数据快照
  pregme::ControlOutput last_output_{};    // 最近一次成功发布的控制量（供状态回显）
  pregme::SimulationClockGuard simulation_clock_guard_{};  // SITL 时钟对齐检查
  eso_interfaces::msg::PregmeStatus status_message_{};     // 复用的状态消息
  eso_interfaces::msg::SafetyEvent event_message_{};       // 复用的事件消息
  std::string fault_reason_{};             // 故障位对应的可读标签

  // ---- 各输入通道的高水位时间戳跟踪器（检测回退/归零）----
  pregme::TimestampTracker local_timestamp_{};
  pregme::TimestampTracker attitude_timestamp_{};
  pregme::TimestampTracker angular_velocity_timestamp_{};
  pregme::TimestampTracker vehicle_status_timestamp_{};
  pregme::TimestampTracker allocator_timestamp_{};
  pregme::TimestampTracker reference_timestamp_{};
  pregme::TimestampTracker native_thrust_timestamp_{};
  pregme::TimestampTracker native_torque_timestamp_{};

  // ---- 各通道最近一次被接受的接收时刻（steady_clock，纳秒）----
  // 0 表示尚未收到；SafetyGate 用它判断超时。
  std::int64_t local_received_ns_{0};
  std::int64_t attitude_received_ns_{0};
  std::int64_t angular_velocity_received_ns_{0};
  std::int64_t vehicle_status_received_ns_{0};
  std::int64_t allocator_status_received_ns_{0};
  std::int64_t reference_received_ns_{0};
  std::int64_t native_thrust_received_ns_{0};
  std::int64_t native_torque_received_ns_{0};

  // ---- 请求状态：由参数回调线程写、tick 线程读，因此用原子量 ----
  std::atomic<std::int64_t> requested_mode_{0};
  std::atomic<bool> enable_control_output_{false};
  bool use_sim_time_{false};

  // ---- 通道健康标志：valid = 消息有效位 + 数值有限；finite 单独保留，
  //      因为 SafetyInputs.values_finite 的检查范围随模式变化 ----
  bool local_valid_{false};
  bool attitude_valid_{false};
  bool angular_velocity_valid_{false};
  bool vehicle_status_valid_{false};
  bool allocator_status_valid_{false};
  bool reference_valid_{false};
  bool native_thrust_valid_{false};
  bool native_torque_valid_{false};
  bool local_finite_{false};
  bool attitude_finite_{false};
  bool angular_velocity_finite_{false};
  bool allocator_finite_{false};
  bool reference_finite_{false};
  bool native_thrust_finite_{false};
  bool native_torque_finite_{false};
  // ---- PX4 状态与重入/复位记录 ----
  bool armed_{false};
  bool offboard_{false};
  bool failsafe_{false};
  bool local_reset_initialized_{false};      // 是否已记录本地位置复位计数器基线
  bool attitude_reset_initialized_{false};   // 是否已记录姿态复位计数器基线
  bool pending_timestamp_event_{false};      // 本拍有待发布的时间戳事件
  bool reentry_required_{false};             // 是否要求先退出主动模式再重进
  bool reentry_clear_requested_{false};      // 参数回调请求解除重入锁存
  std::array<std::uint8_t, 4> local_reset_counters_{};  // xy/z/vxy/vz 四个计数器
  std::uint8_t attitude_reset_counter_{0};
  std::uint32_t reset_count_{0};                          // 累计复位/纪元切换次数
  std::uint32_t pending_reset_reason_flags_{pregme::ResetNone};  // 待发布的原因位
  std::uint32_t last_reset_reason_flags_{pregme::ResetNone};     // 最近一次原因位
  // previous_* 初值刻意取「不可能等于真实值」的值，保证首拍一定发布事件。
  std::uint32_t previous_fault_flags_{~0U};
  pregme::Mode previous_active_mode_{pregme::Mode::FullControl};
  EventSource pending_event_source_{EventSource::Gate};
  std::uint64_t event_sequence_{0};          // 事件序号，接收端据此检测丢失

  // ---- ROS 实体句柄：参数回调、订阅、发布器与定时器 ----
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::ControlAllocatorStatus>::SharedPtr allocator_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr reference_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr native_thrust_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr native_torque_sub_;
  rclcpp::Publisher<eso_interfaces::msg::PregmeStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<eso_interfaces::msg::SafetyEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pregme_control

// 进程入口：构造节点（参数非法会在此抛异常并终止）后进入单线程执行器。
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pregme_control::PregmeControlNode>());
  rclcpp::shutdown();
  return 0;
}
