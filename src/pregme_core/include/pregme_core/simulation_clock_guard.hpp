#pragma once

#include <cstdint>
#include <cstdlib>

// =============================================================================
// SimulationClockGuard：仿真时钟对齐守卫
// -----------------------------------------------------------------------------
// 用途：确认 SITL 中 ROS 仿真时间与 PX4 时间戳确实对齐，作为「允许占位参数
//       进入主动控制」的环境前提之一。
// 原理：每次收到 vehicle_status 时调用 observe()，记录 PX4 时间戳与 ROS 时间
//       的固定偏移；后续帧的偏移必须保持在容差内，且 ROS 时间严格递增。
//       valid() 还要求 use_sim_time 已启用、参数文件显式放行，以及最近一次
//       观察未超时。
// =============================================================================

namespace pregme
{

class SimulationClockGuard
{
public:
  // 记录一对（ROS 时间, PX4 时间戳, 接收时刻）。首帧建立偏移基线；
  // 返回本次偏移是否仍在容差内。ROS 时间非正、PX4 时间戳为 0、接收时刻
  // 非正或 ROS 时间不递增时，直接判定本帧无效。
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

  // 综合判断仿真上下文是否可信，四个条件缺一不可：
  //   1) use_sim_time 已启用；
  //   2) 参数文件显式放行占位参数（explicit_permission）；
  //   3) 已建立偏移基线且最近一对时间戳仍在容差内（pair_valid_）；
  //   4) 最近一次观察距 now 未超过 timeout，且 now 不早于接收时刻。
  bool valid(
    const std::int64_t now_steady_ns, const std::int64_t timeout_ns,
    const bool use_sim_time, const bool explicit_permission) const noexcept
  {
    return use_sim_time && explicit_permission && initialized_ && pair_valid_ &&
           last_received_steady_ns_ > 0 && now_steady_ns >= last_received_steady_ns_ &&
           now_steady_ns - last_received_steady_ns_ <= timeout_ns;
  }

  // 清空全部状态。PX4 时间纪元切换时必须调用，下一帧重新建立偏移基线。
  void reset() noexcept
  {
    initialized_ = false;
    pair_valid_ = false;
    offset_ns_ = 0;
    last_ros_time_ns_ = 0;
    last_received_steady_ns_ = 0;
  }

private:
  bool initialized_{false};            // 是否已建立偏移基线
  bool pair_valid_{false};             // 最近一帧偏移是否在容差内
  std::int64_t offset_ns_{0};          // PX4 时间戳 - ROS 时间的基线偏移
  std::int64_t last_ros_time_ns_{0};   // 上一帧 ROS 时间，用于检查严格递增
  std::int64_t last_received_steady_ns_{0};  // 上一帧接收时刻（单调时钟）
};

}  // namespace pregme
