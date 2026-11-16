#pragma once

#include <cstdint>
#include <cstdlib>

namespace pregme
{

class SimulationClockGuard
{
public:
  bool observe(
    const std::int64_t ros_time_ns, const std::uint64_t px4_timestamp_us,
    const std::int64_t received_steady_ns, const std::int64_t tolerance_ns) noexcept
  {
    if (ros_time_ns <= 0 || px4_timestamp_us == 0U || received_steady_ns <= 0) {
      pair_valid_ = false;
      return false;
    }
    if (last_ros_time_ns_ != 0 && ros_time_ns <= last_ros_time_ns_) {
      pair_valid_ = false;
      return false;
    }
    const auto px4_time_ns = static_cast<std::int64_t>(px4_timestamp_us * 1000ULL);
    const auto offset_ns = px4_time_ns - ros_time_ns;
    if (!initialized_) {
      offset_ns_ = offset_ns;
      initialized_ = true;
    }
    pair_valid_ = std::llabs(offset_ns - offset_ns_) <= tolerance_ns;
    last_ros_time_ns_ = ros_time_ns;
    last_received_steady_ns_ = received_steady_ns;
    return pair_valid_;
  }

  bool valid(
    const std::int64_t now_steady_ns, const std::int64_t timeout_ns,
    const bool use_sim_time, const bool explicit_permission) const noexcept
  {
    return use_sim_time && explicit_permission && initialized_ && pair_valid_ &&
           last_received_steady_ns_ > 0 && now_steady_ns >= last_received_steady_ns_ &&
           now_steady_ns - last_received_steady_ns_ <= timeout_ns;
  }

  void reset() noexcept
  {
    initialized_ = false;
    pair_valid_ = false;
    offset_ns_ = 0;
    last_ros_time_ns_ = 0;
    last_received_steady_ns_ = 0;
  }

private:
  bool initialized_{false};
  bool pair_valid_{false};
  std::int64_t offset_ns_{0};
  std::int64_t last_ros_time_ns_{0};
  std::int64_t last_received_steady_ns_{0};
};

}  // namespace pregme
