#pragma once

#include <cstdint>

// =============================================================================
// TimestampTracker：单通道时间戳高水位跟踪器
// -----------------------------------------------------------------------------
// 每个输入通道各持有一个实例，用来把乱序/重复/回退的时间戳显式地区分开：
//   - First     本通道第一次收到有效时间戳，建立基线；
//   - Advanced  严格递增，真正的新数据；
//   - Duplicate 与上一帧相同，不应重复消费（防止陈旧数据续命）；
//   - InvalidZero 时间戳为 0，视为异常；
//   - Regression 小于高水位，视为时间回退（飞控重启/SITL 重开等）。
// fault_latched_ 具有粘滞性：一旦出现 InvalidZero/Regression 就置位，直到
// 收到严格更新的时间戳才清除；节点据此把故障暴露给安全门。
// =============================================================================

namespace pregme
{

// 时间戳观察结果。timestamp_is_new() 只把 First/Advanced 视为可消费的新数据。
enum class TimestampResult : std::uint8_t {
  First = 0,
  Advanced = 1,
  Duplicate = 2,
  InvalidZero = 3,
  Regression = 4,
};

class TimestampTracker
{
public:
  // 观察一个新的时间戳（微秒），更新高水位并返回分类结果。
  // 注意：Duplicate 不改变状态；InvalidZero/Regression 只锁存故障、不移动高水位。
  TimestampResult observe(std::uint64_t timestamp_us) noexcept
  {
    if (timestamp_us == 0U) {
      fault_latched_ = true;
      return TimestampResult::InvalidZero;
    }
    if (high_water_us_ == 0U) {
      high_water_us_ = timestamp_us;
      fault_latched_ = false;
      return TimestampResult::First;
    }
    if (timestamp_us < high_water_us_) {
      fault_latched_ = true;
      return TimestampResult::Regression;
    }
    if (timestamp_us == high_water_us_) {
      return TimestampResult::Duplicate;
    }
    high_water_us_ = timestamp_us;
    fault_latched_ = false;
    return TimestampResult::Advanced;
  }

  // 清空基线。仅在确认进入新的时间纪元（飞控重启/SITL 重开）时调用，
  // 下一帧消息会重新按 First 建立基线。
  void reset() noexcept
  {
    high_water_us_ = 0U;
    fault_latched_ = false;
  }

  std::uint64_t high_water_us() const noexcept {return high_water_us_;}
  bool fault_latched() const noexcept {return fault_latched_;}

private:
  std::uint64_t high_water_us_{0};   // 已见过的最大时间戳；0 表示尚未建立基线
  bool fault_latched_{false};        // 粘滞故障标志，直到出现更新时间戳才清除
};

constexpr bool timestamp_is_new(const TimestampResult result) noexcept
{
  return result == TimestampResult::First || result == TimestampResult::Advanced;
}

}  // namespace pregme
