// =============================================================================
// RuntimeConfig 校验单元测试
// -----------------------------------------------------------------------------
// 目标：锁定「默认安全集可启动、夹具集必须自洽、启动期参数不可热改」三条
//       规则。测试不依赖 ROS 运行时，只调用纯函数。
// =============================================================================

#include <gtest/gtest.h>

#include "pregme_control/runtime_config.hpp"

namespace
{

// 构造一份自洽的占位夹具配置：身份三元组、质量/几何/惯量、采样周期
// 都取 config/m2_fixture.yaml 的同组数值，任何单项失衡都应导致校验失败。
pregme_control::RuntimeConfig placeholder_config()
{
  pregme_control::RuntimeConfig config{};
  config.has_placeholder = true;
  config.fixture_id = "m2_fixture_v1";
  config.parameter_set_sha256 =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
  config.fixture_watermark = "FIXTURE:m2_fixture_v1:0123456789ab";
  config.body_mass_kg = 5.40;
  config.arm_mass_kg = 2.32;
  config.wheelbase_m = 0.90;
  config.inertia_xx_kg_m2 = 0.48;
  config.inertia_yy_kg_m2 = 0.48;
  config.inertia_zz_kg_m2 = 0.70;
  config.expected_position_period_s = 0.01;
  config.expected_attitude_period_s = 0.01;
  config.minimum_period_s = 0.005;
  config.maximum_period_s = 0.02;
  config.desired_attitude_derivative_cutoff_hz = 20.0;
  return config;
}

// 默认构造即机上安全默认值（Native、输出关闭、无夹具），必须校验通过。
TEST(RuntimeConfig, SafeDefaultsAreValid)
{
  EXPECT_TRUE(pregme_control::validate_runtime_config({}).valid);
}

// 自洽的占位夹具应通过校验（正例，避免校验过严）。
TEST(RuntimeConfig, PlaceholderFixtureIsSelfConsistent)
{
  EXPECT_TRUE(pregme_control::validate_runtime_config(placeholder_config()).valid);
}

// 模式越界和相对话题名都必须被拒绝。
TEST(RuntimeConfig, RejectsInvalidModeRangeAndRelativeTopics)
{
  auto config = pregme_control::RuntimeConfig{};
  config.mode = 4;
  EXPECT_FALSE(pregme_control::validate_runtime_config(config).valid);
  config.mode = 0;
  config.reference_topic = "fmu/out/reference";
  EXPECT_FALSE(pregme_control::validate_runtime_config(config).valid);
}

// 只改惯量不改质量会破坏回转半径自洽性，必须被拒绝。
TEST(RuntimeConfig, RejectsInconsistentFixtureInertia)
{
  auto config = placeholder_config();
  config.inertia_xx_kg_m2 = 0.001;
  EXPECT_FALSE(pregme_control::validate_runtime_config(config).valid);
}

// 水印无法由 id + 哈希前缀重建时视为过期，必须被拒绝。
TEST(RuntimeConfig, RejectsStaleFixtureWatermark)
{
  auto config = placeholder_config();
  config.fixture_watermark = "FIXTURE:m2_fixture_v1:stale";
  EXPECT_FALSE(pregme_control::validate_runtime_config(config).valid);
}

// has_placeholder=false 不能掩盖残留的夹具数值：只要还带着物理参数，
// 就必须提供完整身份三元组。
TEST(RuntimeConfig, PlaceholderFlagCannotHideFixtureValues)
{
  auto config = placeholder_config();
  config.has_placeholder = false;
  config.fixture_id = "none";
  config.parameter_set_sha256 = "none";
  config.fixture_watermark = "none";
  EXPECT_FALSE(pregme_control::validate_runtime_config(config).valid);
}

// 运行期热更新白名单：只有模式和输出总开关，其余参数一律启动期固定。
TEST(RuntimeConfig, OnlyModeAndHardOutputSwitchAreMutable)
{
  EXPECT_TRUE(pregme_control::is_runtime_mutable_parameter("mode"));
  EXPECT_TRUE(pregme_control::is_runtime_mutable_parameter("enable_control_output"));
  EXPECT_FALSE(pregme_control::is_runtime_mutable_parameter("state_timeout_ms"));
  EXPECT_FALSE(pregme_control::is_runtime_mutable_parameter("topics.attitude"));
}

}  // namespace
