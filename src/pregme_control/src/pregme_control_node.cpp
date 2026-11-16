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
    declare_parameters();
    config_ = read_config();
    const auto validation = validate_runtime_config(config_);
    if (!validation.valid) {
      throw std::invalid_argument("invalid PreGME configuration: " + validation.reason);
    }

    requested_mode_.store(config_.mode);
    enable_control_output_.store(config_.enable_control_output);
    use_sim_time_ = get_parameter("use_sim_time").as_bool();
    fault_reason_.reserve(384);
    status_message_.reason.reserve(384);
    event_message_.reason.reserve(384);
    status_message_.fixture_id = config_.fixture_id;
    status_message_.parameter_set_sha256 = config_.parameter_set_sha256;
    status_message_.fixture_watermark = config_.fixture_watermark;
    gate_ = pregme::SafetyGate(pregme::SafetyConfig{
        config_.state_timeout_ms * 1000000LL,
        config_.allocator_timeout_ms * 1000000LL,
        config_.state_timeout_ms * 1000000LL,
        config_.state_timeout_ms * 1000000LL});

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&PregmeControlNode::on_parameters, this, std::placeholders::_1));

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

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / config_.status_rate_hz)),
      std::bind(&PregmeControlNode::tick, this));

    RCLCPP_WARN(
      get_logger(),
      "PreGME formulas are frozen but the executable core is unavailable; modes 2 and 3 remain disabled");
  }

private:
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

  static std::int64_t steady_now_ns() noexcept
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

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

  void declare_parameters()
  {
    declare_parameter<int>("mode", 0);
    declare_parameter<bool>("enable_control_output", false);
    declare_parameter<double>("status_rate_hz", 50.0);
    declare_parameter<int>("state_timeout_ms", 200);
    declare_parameter<int>("allocator_timeout_ms", 250);
    declare_parameter<bool>("allow_placeholder_in_sitl", false);
    declare_parameter<int>("simulation_clock_timeout_ms", 200);
    declare_parameter<int>("simulation_time_tolerance_ms", 100);
    declare_parameter<bool>("fixture.has_placeholder", false);
    declare_parameter<std::string>("fixture.id", "none");
    declare_parameter<std::string>("fixture.parameter_set_sha256", "none");
    declare_parameter<std::string>("fixture.watermark", "none");
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

  static bool owned_parameter(const std::string & name)
  {
    return name == "mode" || name == "enable_control_output" || name == "status_rate_hz" ||
           name == "state_timeout_ms" || name == "allocator_timeout_ms" ||
           name == "allow_placeholder_in_sitl" || name == "simulation_clock_timeout_ms" ||
           name == "simulation_time_tolerance_ms" || name == "use_sim_time" ||
           name.rfind("topics.", 0) == 0 || name.rfind("fixture.", 0) == 0;
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result{};
    result.successful = false;
    auto mode = requested_mode_.load();
    auto enable = enable_control_output_.load();

    for (const auto & parameter : parameters) {
      if (owned_parameter(parameter.get_name()) &&
        !is_runtime_mutable_parameter(parameter.get_name()))
      {
        result.reason = parameter.get_name() + " is startup-only; restart the node to change it";
        return result;
      }
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

    requested_mode_.store(mode);
    enable_control_output_.store(enable);
    if (mode <= static_cast<std::int64_t>(pregme::Mode::Shadow)) {
      reentry_clear_requested_ = true;
    }
    result.successful = true;
    result.reason = "ok";
    return result;
  }

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

  bool any_timestamp_fault() const noexcept
  {
    return local_timestamp_.fault_latched() || attitude_timestamp_.fault_latched() ||
           angular_velocity_timestamp_.fault_latched() || vehicle_status_timestamp_.fault_latched() ||
           allocator_timestamp_.fault_latched() || reference_timestamp_.fault_latched() ||
           native_thrust_timestamp_.fault_latched() || native_torque_timestamp_.fault_latched();
  }

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

    vehicle_status_timestamp_.observe(vehicle_status_timestamp_us);
    pending_timestamp_event_ = true;
    pending_event_source_ = EventSource::VehicleStatus;
    pending_reset_reason_flags_ |= pregme::ResetPx4TimeEpoch;
    last_reset_reason_flags_ = pending_reset_reason_flags_;
    ++reset_count_;
    reentry_required_ = true;
  }

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
    armed_ = message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    offboard_ = message->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    failsafe_ = message->failsafe;
    simulation_clock_guard_.observe(
      get_clock()->now().nanoseconds(), message->timestamp, received_ns,
      config_.simulation_time_tolerance_ms * 1000000LL);
  }

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

  static void describe_faults(const std::uint32_t faults, std::string & reason)
  {
    reason.clear();
    if (faults == pregme::FaultNone) {
      reason.append("ok");
      return;
    }
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

  void tick()
  {
    const auto requested_mode = requested_mode_.load();
    if (reentry_clear_requested_ && requested_mode <= static_cast<std::int64_t>(pregme::Mode::Shadow)) {
      reentry_required_ = false;
      reentry_clear_requested_ = false;
    }

    const bool shadow_or_active = requested_mode >= static_cast<std::int64_t>(pregme::Mode::Shadow);
    const auto now_ns = steady_now_ns();
    const bool simulation_context_valid = simulation_clock_guard_.valid(
      now_ns, config_.simulation_clock_timeout_ms * 1000000LL,
      use_sim_time_, config_.allow_placeholder_in_sitl);

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

    auto decision = gate_.evaluate(
      requested_mode, enable_control_output_.load(), pregme::algorithm_ready(), inputs);
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

    pending_timestamp_event_ = false;
    pending_reset_reason_flags_ = pregme::ResetNone;

    const pregme::ControlOutput output{};
    if (decision.publish_control && output.valid) {
      publish_control(output);
    }
  }

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

  RuntimeConfig config_{};
  pregme::SafetyGate gate_{};
  pregme::InputState state_{};
  pregme::ControlOutput last_output_{};
  pregme::SimulationClockGuard simulation_clock_guard_{};
  eso_interfaces::msg::PregmeStatus status_message_{};
  eso_interfaces::msg::SafetyEvent event_message_{};
  std::string fault_reason_{};

  pregme::TimestampTracker local_timestamp_{};
  pregme::TimestampTracker attitude_timestamp_{};
  pregme::TimestampTracker angular_velocity_timestamp_{};
  pregme::TimestampTracker vehicle_status_timestamp_{};
  pregme::TimestampTracker allocator_timestamp_{};
  pregme::TimestampTracker reference_timestamp_{};
  pregme::TimestampTracker native_thrust_timestamp_{};
  pregme::TimestampTracker native_torque_timestamp_{};

  std::int64_t local_received_ns_{0};
  std::int64_t attitude_received_ns_{0};
  std::int64_t angular_velocity_received_ns_{0};
  std::int64_t vehicle_status_received_ns_{0};
  std::int64_t allocator_status_received_ns_{0};
  std::int64_t reference_received_ns_{0};
  std::int64_t native_thrust_received_ns_{0};
  std::int64_t native_torque_received_ns_{0};

  std::atomic<std::int64_t> requested_mode_{0};
  std::atomic<bool> enable_control_output_{false};
  bool use_sim_time_{false};
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
  bool armed_{false};
  bool offboard_{false};
  bool failsafe_{false};
  bool local_reset_initialized_{false};
  bool attitude_reset_initialized_{false};
  bool pending_timestamp_event_{false};
  bool reentry_required_{false};
  bool reentry_clear_requested_{false};
  std::array<std::uint8_t, 4> local_reset_counters_{};
  std::uint8_t attitude_reset_counter_{0};
  std::uint32_t reset_count_{0};
  std::uint32_t pending_reset_reason_flags_{pregme::ResetNone};
  std::uint32_t last_reset_reason_flags_{pregme::ResetNone};
  std::uint32_t previous_fault_flags_{~0U};
  pregme::Mode previous_active_mode_{pregme::Mode::FullControl};
  EventSource pending_event_source_{EventSource::Gate};
  std::uint64_t event_sequence_{0};

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pregme_control::PregmeControlNode>());
  rclcpp::shutdown();
  return 0;
}
