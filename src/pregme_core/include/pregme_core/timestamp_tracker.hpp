#pragma once

#include <cstdint>

namespace pregme
{

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

  void reset() noexcept
  {
    high_water_us_ = 0U;
    fault_latched_ = false;
  }

  std::uint64_t high_water_us() const noexcept {return high_water_us_;}
  bool fault_latched() const noexcept {return fault_latched_;}

private:
  std::uint64_t high_water_us_{0};
  bool fault_latched_{false};
};

constexpr bool timestamp_is_new(const TimestampResult result) noexcept
{
  return result == TimestampResult::First || result == TimestampResult::Advanced;
}

}  // namespace pregme
