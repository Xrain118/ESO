#include <gtest/gtest.h>

#include "pregme_control/runtime_config.hpp"

namespace
{

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

TEST(RuntimeConfig, SafeDefaultsAreValid)
{
  EXPECT_TRUE(pregme_control::validate_runtime_config({}).valid);
}

TEST(RuntimeConfig, PlaceholderFixtureIsSelfConsistent)
{
  EXPECT_TRUE(pregme_control::validate_runtime_config(placeholder_config()).valid);
}

TEST(RuntimeConfig, RejectsInvalidModeRangeAndRelativeTopics)
{
  auto config = pregme_control::RuntimeConfig{};
  config.mode = 4;
  EXPECT_FALSE(pregme_control::validate_runtime_config(config).valid);
  config.mode = 0;
  config.reference_topic = "fmu/out/reference";
  EXPECT_FALSE(pregme_control::validate_runtime_config(config).valid);
}

TEST(RuntimeConfig, RejectsInconsistentFixtureInertia)
{
  auto config = placeholder_config();
  config.inertia_xx_kg_m2 = 0.001;
  EXPECT_FALSE(pregme_control::validate_runtime_config(config).valid);
}

TEST(RuntimeConfig, RejectsStaleFixtureWatermark)
{
  auto config = placeholder_config();
  config.fixture_watermark = "FIXTURE:m2_fixture_v1:stale";
  EXPECT_FALSE(pregme_control::validate_runtime_config(config).valid);
}

TEST(RuntimeConfig, PlaceholderFlagCannotHideFixtureValues)
{
  auto config = placeholder_config();
  config.has_placeholder = false;
  config.fixture_id = "none";
  config.parameter_set_sha256 = "none";
  config.fixture_watermark = "none";
  EXPECT_FALSE(pregme_control::validate_runtime_config(config).valid);
}

TEST(RuntimeConfig, OnlyModeAndHardOutputSwitchAreMutable)
{
  EXPECT_TRUE(pregme_control::is_runtime_mutable_parameter("mode"));
  EXPECT_TRUE(pregme_control::is_runtime_mutable_parameter("enable_control_output"));
  EXPECT_FALSE(pregme_control::is_runtime_mutable_parameter("state_timeout_ms"));
  EXPECT_FALSE(pregme_control::is_runtime_mutable_parameter("topics.attitude"));
}

}  // namespace
