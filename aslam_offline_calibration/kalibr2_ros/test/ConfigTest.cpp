
#include <filesystem>

#include <gtest/gtest.h>
#include <kalibr2_ros/Config.hpp>

namespace {

using namespace kalibr2::ros;

TEST(ConfigFromYAMLTest, InvalidBagFile) {
  std::string invalid_bag_path = TEST_DATA_DIR "/invalid_bag.mcap";
  EXPECT_THROW({ CalibrationConfig config = ConfigFromYaml(invalid_bag_path); }, std::runtime_error);
}

// TODO(frneer): Add tests that don't depend on an existing rosbag or add a mock
TEST(ConfigFromYAMLTest, MissingTargetConfig) {
  auto bag_path = TEST_DATA_DIR "/rosbag2_2025_06_11-12_00_21_0.mcap";
  if (!std::filesystem::exists(bag_path)) {
    GTEST_SKIP() << "Bag file does not exist: " << bag_path;
  }
  std::string missing_target_yaml_path = TEST_DATA_DIR "/missing_target_config.yaml";
  EXPECT_THROW({ CalibrationConfig config = ConfigFromYaml(missing_target_yaml_path); }, std::runtime_error);
}

// TODO(frneer): Add tests that don't depend on an existing rosbag or add a mock
TEST(ConfigFromYAMLTest, WorkingYAML) {
  auto bag_path = TEST_DATA_DIR "/rosbag2_2025_06_11-12_00_21_0.mcap";
  if (!std::filesystem::exists(bag_path)) {
    GTEST_SKIP() << "Bag file does not exist: " << bag_path;
  }
  std::string valid_yaml_path = TEST_DATA_DIR "/valid_config.yaml";

  CalibrationConfig config = ConfigFromYaml(valid_yaml_path);
  EXPECT_EQ(config.cameras.size(), 2);
  ASSERT_NE(config.target, nullptr);
  for (const auto& camera_config : config.cameras) {
    EXPECT_NE(camera_config.reader, nullptr);
    EXPECT_EQ(camera_config.model, "pinhole-radtan");
    EXPECT_EQ(camera_config.focal_length.value(), 881.0);
  }
}

}  // namespace
