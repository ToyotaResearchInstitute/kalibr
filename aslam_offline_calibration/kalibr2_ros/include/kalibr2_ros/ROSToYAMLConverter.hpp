#pragma once

// Provide utilities to convert ROS messages to YAML format

#include <filesystem>
#include <fstream>

#include <geometry_msgs/msg/detail/transform_stamped__traits.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kalibr2/CameraCalibrator.hpp>
#include <kalibr2_ros/KalibrToROSModelConverter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/detail/camera_info__traits.hpp>
#include <yaml-cpp/yaml.h>

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

void CalibratorToYAML(const boost::shared_ptr<CameraCalibratorBase>& calibrator, const std::string& model,
                      const std::string& frame_id, size_t width, size_t height,
                      const std::filesystem::path& yaml_file) {
  // Get the camera geometry which stores the intrinsic parameters
  auto camera_geometry = calibrator->camera_geometry();

  // Get projection parameters (intrinsics)
  Eigen::MatrixXd projection_params;
  camera_geometry->getParameters(projection_params, true, false, false);

  // Get distortion parameters
  Eigen::MatrixXd distortion_params;
  camera_geometry->getParameters(distortion_params, false, true, false);

  // Create CameraInfo message
  sensor_msgs::msg::CameraInfo camera_info;

  // Set frame ID
  camera_info.header.frame_id = frame_id;

  // Set image dimensions
  camera_info.width = width;
  camera_info.height = height;

  // Assuming pinhole model: projection_params contains [fx, fy, cx, cy]
  // K = [fx  0  cx]
  //     [ 0 fy  cy]
  //     [ 0  0   1]
  if (projection_params.size() >= 4) {
    camera_info.k[0] = projection_params(0);  // fx
    camera_info.k[1] = 0.0;
    camera_info.k[2] = projection_params(2);  // cx
    camera_info.k[3] = 0.0;
    camera_info.k[4] = projection_params(1);  // fy
    camera_info.k[5] = projection_params(3);  // cy
    camera_info.k[6] = 0.0;
    camera_info.k[7] = 0.0;
    camera_info.k[8] = 1.0;

    // Also set P matrix (projection matrix) - for rectified images, typically P = K
    camera_info.p[0] = projection_params(0);  // fx
    camera_info.p[1] = 0.0;
    camera_info.p[2] = projection_params(2);  // cx
    camera_info.p[3] = 0.0;
    camera_info.p[4] = 0.0;
    camera_info.p[5] = projection_params(1);  // fy
    camera_info.p[6] = projection_params(3);  // cy
    camera_info.p[7] = 0.0;
    camera_info.p[8] = 0.0;
    camera_info.p[9] = 0.0;
    camera_info.p[10] = 1.0;
    camera_info.p[11] = 0.0;
  }

  // Map kalibr model to ROS distortion model
  std::string ros_distortion_model = ToROSDistortionModel(model);

  // Set distortion model and coefficients
  camera_info.distortion_model = ros_distortion_model;
  if (distortion_params.size() > 0) {
    camera_info.d.resize(distortion_params.size());
    for (int i = 0; i < distortion_params.size(); ++i) {
      camera_info.d[i] = distortion_params(i);
    }
  } else {
    camera_info.d.resize(0);
  }

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
