#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <eso_interfaces/msg/pregme_status.hpp>
#include <eso_interfaces/msg/safety_event.hpp>
#include <pregme_core/safety_gate.hpp>
#include <pregme_core/types.hpp>
#include <px4_msgs/msg/control_allocator_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

namespace pregme_control
{

using namespace std::chrono_literals;

class PregmeControlNode final : public rclcpp::Node
{
public:
  PregmeControlNode()
  : Node("pregme_control"),
    gate_(pregme::SafetyConfig{
      static_cast<std::int64_t>(declare_parameter<int>("state_timeout_ms", 200)) * 1000000LL,
      static_cast<std::int64_t>(declare_parameter<int>("allocator_timeout_ms", 250)) * 1000000LL})
  {
    declare_parameter<int>("mode", 0);
    declare_parameter<bool>("enable_control_output", false);
    declare_parameter<double>("status_rate_hz", 50.0);
    declare_parameter<std::string>("topics.local_position", "/fmu/out/vehicle_local_position");
    declare_parameter<std::string>("topics.attitude", "/fmu/out/vehicle_attitude");
    declare_parameter<std::string>("topics.angular_velocity", "/fmu/out/vehicle_angular_velocity");
    declare_parameter<std::string>("topics.vehicle_status", "/fmu/out/vehicle_status_v1");
    declare_parameter<std::string>("topics.allocator_status", "/fmu/out/control_allocator_status");

    const auto sensor_qos = rclcpp::SensorDataQoS();
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      get_parameter("topics.local_position").as_string(), sensor_qos,
      std::bind(&PregmeControlNode::on_local_position, this, std::placeholders::_1));
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      get_parameter("topics.attitude").as_string(), sensor_qos,
      std::bind(&PregmeControlNode::on_attitude, this, std::placeholders::_1));
    angular_velocity_sub_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
      get_parameter("topics.angular_velocity").as_string(), sensor_qos,
      std::bind(&PregmeControlNode::on_angular_velocity, this, std::placeholders::_1));
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      get_parameter("topics.vehicle_status").as_string(), sensor_qos,
      std::bind(&PregmeControlNode::on_vehicle_status, this, std::placeholders::_1));
    allocator_status_sub_ = create_subscription<px4_msgs::msg::ControlAllocatorStatus>(
      get_parameter("topics.allocator_status").as_string(), sensor_qos,
      std::bind(&PregmeControlNode::on_allocator_status, this, std::placeholders::_1));

    status_pub_ = create_publisher<eso_interfaces::msg::PregmeStatus>("/eso/pregme/status", 10);
    event_pub_ = create_publisher<eso_interfaces::msg::SafetyEvent>("/eso/pregme/safety_event", 10);

    // These are the only flight-control publishers owned by ESO. The safety gate
    // prevents them from publishing until the formula core is validated.
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    thrust_pub_ = create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
      "/fmu/in/vehicle_thrust_setpoint", 10);
    torque_pub_ = create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
      "/fmu/in/vehicle_torque_setpoint", 10);

    const double requested_rate = get_parameter("status_rate_hz").as_double();
    const double rate = std::clamp(requested_rate, 1.0, 100.0);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)),
      std::bind(&PregmeControlNode::tick, this));

    RCLCPP_WARN(
      get_logger(),
      "PreGME formula core is not implemented: modes 2 and 3 are hard-disabled");
  }

private:
  static std::int64_t steady_now_ns() noexcept
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static bool finite3(const std::array<float, 3> & values) noexcept
  {
    return std::all_of(values.begin(), values.end(), [](float value) {return std::isfinite(value);});
  }

  static bool finite4(const std::array<float, 4> & values) noexcept
  {
    return std::all_of(values.begin(), values.end(), [](float value) {return std::isfinite(value);});
  }

  bool update_timestamp(const std::uint64_t sample, std::uint64_t & previous) noexcept
  {
    if (sample == 0U || (previous != 0U && sample < previous)) {
      timestamp_monotonic_ = false;
      return false;
    }
    previous = sample;
    return true;
  }

  void on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr message)
  {
    update_timestamp(message->timestamp_sample, last_local_sample_);
    local_received_ns_ = steady_now_ns();
    state_.timestamp_sample_us = std::max(state_.timestamp_sample_us, message->timestamp_sample);
    state_.position_ned = {message->x, message->y, message->z};
    state_.velocity_ned = {message->vx, message->vy, message->vz};
    local_valid_ = message->xy_valid && message->z_valid && message->v_xy_valid &&
      message->v_z_valid && finite3(state_.position_ned) && finite3(state_.velocity_ned);

    const std::array<std::uint8_t, 4> reset_counters{
      message->xy_reset_counter, message->z_reset_counter,
      message->vxy_reset_counter, message->vz_reset_counter};
    if (local_reset_initialized_ && reset_counters != local_reset_counters_) {
      estimator_reset_ = true;
      ++reset_count_;
    }
    local_reset_counters_ = reset_counters;
    local_reset_initialized_ = true;
  }

  void on_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr message)
  {
    update_timestamp(message->timestamp_sample, last_attitude_sample_);
    attitude_received_ns_ = steady_now_ns();
    state_.timestamp_sample_us = std::max(state_.timestamp_sample_us, message->timestamp_sample);
    std::copy(message->q.begin(), message->q.end(), state_.attitude_wxyz.begin());
    const float norm_squared =
      state_.attitude_wxyz[0] * state_.attitude_wxyz[0] +
      state_.attitude_wxyz[1] * state_.attitude_wxyz[1] +
      state_.attitude_wxyz[2] * state_.attitude_wxyz[2] +
      state_.attitude_wxyz[3] * state_.attitude_wxyz[3];
    attitude_valid_ = finite4(state_.attitude_wxyz) && norm_squared > 0.25F;

    if (attitude_reset_initialized_ && message->quat_reset_counter != attitude_reset_counter_) {
      estimator_reset_ = true;
      ++reset_count_;
    }
    attitude_reset_counter_ = message->quat_reset_counter;
    attitude_reset_initialized_ = true;
  }

  void on_angular_velocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr message)
  {
    update_timestamp(message->timestamp_sample, last_angular_velocity_sample_);
    angular_velocity_received_ns_ = steady_now_ns();
    state_.timestamp_sample_us = std::max(state_.timestamp_sample_us, message->timestamp_sample);
    std::copy(message->xyz.begin(), message->xyz.end(), state_.angular_velocity_frd.begin());
    angular_velocity_valid_ = finite3(state_.angular_velocity_frd);
  }

  void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr message)
  {
    vehicle_status_received_ns_ = steady_now_ns();
    vehicle_status_valid_ = !message->failsafe;
    armed_ = message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    offboard_ = message->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  void on_allocator_status(const px4_msgs::msg::ControlAllocatorStatus::SharedPtr message)
  {
    allocator_status_received_ns_ = steady_now_ns();
    std::array<float, 3> unallocated_thrust{};
    std::array<float, 3> unallocated_torque{};
    std::copy(
      message->unallocated_thrust.begin(), message->unallocated_thrust.end(),
      unallocated_thrust.begin());
    std::copy(
      message->unallocated_torque.begin(), message->unallocated_torque.end(),
      unallocated_torque.begin());
    allocator_status_valid_ = finite3(unallocated_thrust) && finite3(unallocated_torque) &&
      message->handled_motor_failure_mask == 0U;
  }

  std::uint32_t validity_flags() const noexcept
  {
    std::uint32_t flags = 0U;
    if (local_valid_) {flags |= eso_interfaces::msg::PregmeStatus::VALID_LOCAL_POSITION;}
    if (attitude_valid_) {flags |= eso_interfaces::msg::PregmeStatus::VALID_ATTITUDE;}
    if (angular_velocity_valid_) {flags |= eso_interfaces::msg::PregmeStatus::VALID_ANGULAR_VELOCITY;}
    if (vehicle_status_valid_) {flags |= eso_interfaces::msg::PregmeStatus::VALID_VEHICLE_STATUS;}
    if (allocator_status_valid_) {flags |= eso_interfaces::msg::PregmeStatus::VALID_ALLOCATOR_STATUS;}
    return flags;
  }

  static std::string describe_faults(const std::uint32_t faults)
  {
    if (faults == pregme::FaultNone) {return "ok";}
    std::ostringstream reason;
    const auto add = [&reason](const char * text) {
        if (reason.tellp() > 0) {reason << ',';}
        reason << text;
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
    return reason.str();
  }

  void tick()
  {
    const auto requested_mode = get_parameter("mode").as_int();
    const bool active_mode_requested = requested_mode >= 2;
    pregme::SafetyInputs inputs{};
    inputs.now_ns = steady_now_ns();
    inputs.local_position_received_ns = local_received_ns_;
    inputs.attitude_received_ns = attitude_received_ns_;
    inputs.angular_velocity_received_ns = angular_velocity_received_ns_;
    inputs.vehicle_status_received_ns = vehicle_status_received_ns_;
    inputs.allocator_status_received_ns = allocator_status_received_ns_;
    inputs.local_position_valid = local_valid_;
    inputs.attitude_valid = attitude_valid_;
    inputs.angular_velocity_valid = angular_velocity_valid_;
    inputs.vehicle_status_valid = vehicle_status_valid_ &&
      (!active_mode_requested || (armed_ && offboard_));
    inputs.allocator_status_valid = allocator_status_valid_;
    inputs.timestamp_monotonic = timestamp_monotonic_;
    inputs.estimator_reset = estimator_reset_;
    inputs.values_finite = local_valid_ && attitude_valid_ && angular_velocity_valid_;

    const auto decision = gate_.evaluate(
      requested_mode, get_parameter("enable_control_output").as_bool(),
      pregme::algorithm_ready(), inputs);
    publish_status(decision);

    if (decision.fault_flags != previous_fault_flags_ ||
      decision.active_mode != previous_active_mode_)
    {
      publish_event(decision);
      previous_fault_flags_ = decision.fault_flags;
      previous_active_mode_ = decision.active_mode;
    }

    // The formula core currently cannot produce a valid ControlOutput. Keep the
    // publication branch explicit so future code must pass both safety gates.
    const pregme::ControlOutput output{};
    if (decision.publish_control && output.valid) {
      publish_control(output);
    }
  }

  void publish_status(const pregme::SafetyDecision & decision)
  {
    eso_interfaces::msg::PregmeStatus message{};
    message.timestamp = static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
    message.timestamp_sample = state_.timestamp_sample_us;
    message.requested_mode = static_cast<std::uint8_t>(decision.requested_mode);
    message.active_mode = static_cast<std::uint8_t>(decision.active_mode);
    message.algorithm_ready = pregme::algorithm_ready();
    message.input_valid = decision.inputs_valid;
    message.output_enabled = decision.publish_control;
    message.validity_flags = validity_flags();
    message.fault_flags = decision.fault_flags;
    message.reset_count = reset_count_;
    message.reason = describe_faults(decision.fault_flags);
    message.position_ned = state_.position_ned;
    message.velocity_ned = state_.velocity_ned;
    message.attitude_wxyz = state_.attitude_wxyz;
    message.angular_velocity_frd = state_.angular_velocity_frd;
    message.thrust_setpoint_frd = last_output_.thrust_frd;
    message.torque_setpoint_frd = last_output_.torque_frd;
    status_pub_->publish(message);
  }

  void publish_event(const pregme::SafetyDecision & decision)
  {
    eso_interfaces::msg::SafetyEvent message{};
    message.timestamp = static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
    message.timestamp_sample = state_.timestamp_sample_us;
    message.level = decision.fault_flags == pregme::FaultNone ?
      eso_interfaces::msg::SafetyEvent::LEVEL_INFO :
      eso_interfaces::msg::SafetyEvent::LEVEL_WARNING;
    message.requested_mode = static_cast<std::uint8_t>(decision.requested_mode);
    message.active_mode = static_cast<std::uint8_t>(decision.active_mode);
    message.fault_flags = decision.fault_flags;
    message.reason = describe_faults(decision.fault_flags);
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
    thrust.timestamp_sample = state_.timestamp_sample_us;
    thrust.xyz = output.thrust_frd;
    thrust_pub_->publish(thrust);

    px4_msgs::msg::VehicleTorqueSetpoint torque{};
    torque.timestamp = timestamp;
    torque.timestamp_sample = state_.timestamp_sample_us;
    torque.xyz = output.torque_frd;
    torque_pub_->publish(torque);
    last_output_ = output;
  }

  pregme::SafetyGate gate_;
  pregme::InputState state_{};
  pregme::ControlOutput last_output_{};

  std::int64_t local_received_ns_{0};
  std::int64_t attitude_received_ns_{0};
  std::int64_t angular_velocity_received_ns_{0};
  std::int64_t vehicle_status_received_ns_{0};
  std::int64_t allocator_status_received_ns_{0};
  std::uint64_t last_local_sample_{0};
  std::uint64_t last_attitude_sample_{0};
  std::uint64_t last_angular_velocity_sample_{0};

  bool local_valid_{false};
  bool attitude_valid_{false};
  bool angular_velocity_valid_{false};
  bool vehicle_status_valid_{false};
  bool allocator_status_valid_{false};
  bool armed_{false};
  bool offboard_{false};
  bool timestamp_monotonic_{true};
  bool estimator_reset_{false};
  bool local_reset_initialized_{false};
  bool attitude_reset_initialized_{false};
  std::array<std::uint8_t, 4> local_reset_counters_{};
  std::uint8_t attitude_reset_counter_{0};
  std::uint32_t reset_count_{0};
  std::uint32_t previous_fault_flags_{~0U};
  pregme::Mode previous_active_mode_{pregme::Mode::FullControl};

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::ControlAllocatorStatus>::SharedPtr allocator_status_sub_;
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
