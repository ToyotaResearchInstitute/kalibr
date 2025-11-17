#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <aslam/cameras/GridCalibrationTargetBase.hpp>
#include <boost/shared_ptr.hpp>
#include <kalibr2/Image.hpp>

namespace kalibr2::ros {

struct CameraConfig {
  std::unique_ptr<kalibr2::ImageReader> reader;
  std::string model;
  std::optional<double> focal_length;  // Optional focal length for calibration
};

struct CalibrationConfig {
  std::vector<CameraConfig> cameras;
  boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target;
};

/// @brief Load calibration configuration from a YAML file
/// @param yaml_path Path to the YAML configuration file
/// @return CalibrationConfig populated from the YAML file
/// @throws std::runtime_error if the file cannot be read or parsed
CalibrationConfig ConfigFromYaml(const std::string& yaml_path);

}  // namespace kalibr2::ros
