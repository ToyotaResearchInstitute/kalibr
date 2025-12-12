
#include <filesystem>
#include <fstream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <kalibr2_ros/ROSToYAMLConverter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace {

using namespace kalibr2::ros;

TEST(ROSToYAMLConverterTest, CameraInfoToYAML) {
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.width = 640;
  camera_info.height = 480;
  camera_info.distortion_model = "plumb_bob";
  camera_info.d = {0.1, -0.25, 0.001, 0.0005, 0.0};
  camera_info.k = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};

  // Create a file to write the YAML output
  std::filesystem::path output_file = std::filesystem::temp_directory_path() / "camera_info.yaml";
  cameraInfoToYAML(camera_info, output_file);

  // Verify file was created
  ASSERT_TRUE(std::filesystem::exists(output_file));
}

TEST(ROSToYAMLConverterTest, TransformStampedToYAML) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "camera";
  transform.header.stamp.sec = 123;
  transform.header.stamp.nanosec = 456789000;

  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;

  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  // Create a file to write the YAML output
  std::filesystem::path output_file = std::filesystem::temp_directory_path() / "transform_stamped.yaml";
  transformStampedToYAML(transform, output_file);

  // Verify file was created
  ASSERT_TRUE(std::filesystem::exists(output_file));
}

}  // namespace
