#pragma once

// Provide utilities to convert ROS messages to YAML format

#include <filesystem>
#include <fstream>

#include <geometry_msgs/msg/detail/transform_stamped__traits.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kalibr2/CameraCalibrator.hpp>
#include <kalibr2_ros/KalibrToROSConverter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/detail/camera_info__traits.hpp>
#include <tf2_msgs/msg/detail/tf_message__traits.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <yaml-cpp/yaml.h>

namespace kalibr2::ros {

/// @brief Write a CameraInfo message to a YAML file
/// @param camera_info The CameraInfo message to export
/// @param yaml_file The output YAML file path
void cameraInfoToYAML(const sensor_msgs::msg::CameraInfo& camera_info, const std::filesystem::path& yaml_file);

/// @brief Write a TransformStamped message to a YAML file
/// @param transform The TransformStamped message to export
/// @param yaml_file The output YAML file path
void transformStampedToYAML(const geometry_msgs::msg::TransformStamped& transform,
                            const std::filesystem::path& yaml_file);

/// @brief Write a TFMessage to a YAML file
/// @param tf_message The TFMessage to export
/// @param yaml_file The output YAML file path
void tfMessageToYAML(const tf2_msgs::msg::TFMessage& tf_message, const std::filesystem::path& yaml_file);

/// @brief Convert camera calibration results to CameraInfo YAML file
/// @param calibrator The camera calibrator containing calibration results
/// @param model The Kalibr2 camera model string
/// @param frame_id The frame ID for the camera
/// @param width The image width
/// @param height The image height
/// @param yaml_file The output YAML file path
void CalibratorToYAML(const boost::shared_ptr<CameraCalibratorBase>& calibrator, const std::string& model,
                      const std::string& frame_id, size_t width, size_t height, const std::filesystem::path& yaml_file);

}  // namespace kalibr2::ros
