#include <kalibr2_ros/ROSToYAMLConverter.hpp>

namespace kalibr2::ros {

void cameraInfoToYAML(const sensor_msgs::msg::CameraInfo& camera_info, const std::filesystem::path& yaml_file) {
  std::ofstream ofs(yaml_file);
  if (!ofs.is_open()) {
    throw std::runtime_error("Failed to open file: " + yaml_file.string());
  }
  sensor_msgs::msg::to_block_style_yaml(camera_info, ofs);
}

void transformStampedToYAML(const geometry_msgs::msg::TransformStamped& transform,
                            const std::filesystem::path& yaml_file) {
  std::ofstream ofs(yaml_file);
  if (!ofs.is_open()) {
    throw std::runtime_error("Failed to open file: " + yaml_file.string());
  }
  geometry_msgs::msg::to_block_style_yaml(transform, ofs);
}

void tfMessageToYAML(const tf2_msgs::msg::TFMessage& tf_message, const std::filesystem::path& yaml_file) {
  std::ofstream ofs(yaml_file);
  if (!ofs.is_open()) {
    throw std::runtime_error("Failed to open file: " + yaml_file.string());
  }
  tf2_msgs::msg::to_block_style_yaml(tf_message, ofs);
}

void CalibratorToYAML(const boost::shared_ptr<CameraCalibratorBase>& calibrator, const std::string& model,
                      const std::string& frame_id, size_t width, size_t height,
                      const std::filesystem::path& yaml_file) {
  const auto p = calibrator->GetCameraInfoParams();

  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header.frame_id = frame_id;
  camera_info.width = width;
  camera_info.height = height;

  // K = [fx  0  cx]
  //     [ 0 fy  cy]
  //     [ 0  0   1]
  camera_info.k[0] = p.fx;
  camera_info.k[2] = p.cx;
  camera_info.k[4] = p.fy;
  camera_info.k[5] = p.cy;
  camera_info.k[8] = 1.0;

  // P = [fx  0  cx  0]
  //     [ 0  fy cy  0]
  //     [ 0  0   1  0]
  camera_info.p[0] = p.fx;
  camera_info.p[2] = p.cx;
  camera_info.p[5] = p.fy;
  camera_info.p[6] = p.cy;
  camera_info.p[10] = 1.0;

  camera_info.distortion_model = ToROSDistortionModel(model);
  camera_info.d = p.d;

  // Set rectification matrix to identity, no rectification by default.
  camera_info.r[0] = 1.0;
  camera_info.r[1] = 0.0;
  camera_info.r[2] = 0.0;
  camera_info.r[3] = 0.0;
  camera_info.r[4] = 1.0;
  camera_info.r[5] = 0.0;
  camera_info.r[6] = 0.0;
  camera_info.r[7] = 0.0;
  camera_info.r[8] = 1.0;

  cameraInfoToYAML(camera_info, yaml_file);
}

}  // namespace kalibr2::ros
