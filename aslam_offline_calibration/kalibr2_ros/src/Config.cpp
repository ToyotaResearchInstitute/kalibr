#include <stdexcept>

#include <aslam/cameras/GridCalibrationTargetAprilgridv2.hpp>
#include <kalibr2_ros/BagReader.hpp>
#include <kalibr2_ros/Config.hpp>
#include <yaml-cpp/yaml.h>

namespace kalibr2::ros {

namespace {

boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> ParseTarget(const YAML::Node& board_config) {
  if (!board_config) {
    throw std::runtime_error("Board configuration is missing");
  }

  std::string target_type = board_config["target_type"].as<std::string>();

  if (target_type == "aprilgrid") {
    int tag_rows = board_config["tagRows"].as<int>();
    int tag_cols = board_config["tagCols"].as<int>();
    double tag_size = board_config["tagSize"].as<double>();
    double tag_spacing = board_config["tagSpacing"].as<double>();

    auto target_options = aslam::cameras::GridCalibrationTargetAprilgridv2::AprilgridOptionsv2();
    target_options.detectorParameters->markerBorderBits = 2;

    return boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgridv2>(tag_rows, tag_cols, tag_size,
                                                                                tag_spacing, target_options);
  } else {
    throw std::runtime_error("Unsupported target_type: " + target_type);
  }
}

CameraConfig ParseCamera(const std::string& camera_name, const YAML::Node& camera_config) {
  if (!camera_config) {
    throw std::runtime_error("Camera configuration is missing");
  }

  std::string model = camera_config["model"].as<std::string>();

  std::optional<double> focal_length;
  if (camera_config["focal_length_fallback"]) {
    focal_length = camera_config["focal_length_fallback"].as<double>();
  }

  const auto& source = camera_config["source"];
  if (!source) {
    throw std::runtime_error("Camera configuration missing 'source' section");
  }

  std::string rosbag_path = source["rosbag_path"].as<std::string>();
  std::string topic = source["topic"].as<std::string>();

  auto reader = BagImageReaderFactory::create(rosbag_path, topic);

  return CameraConfig{camera_name, std::move(reader), model, focal_length};
}

}  // anonymous namespace

CalibrationConfig ConfigFromYaml(const std::string& yaml_path) {
  YAML::Node config_yaml;
  try {
    config_yaml = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load YAML file '" + yaml_path + "': " + e.what());
  }

  CalibrationConfig config;

  // Parse target board configuration
  if (!config_yaml["board"]) {
    throw std::runtime_error("YAML file missing 'board' section");
  }
  config.target = ParseTarget(config_yaml["board"]);

  // Parse cameras configuration
  if (!config_yaml["cameras"]) {
    throw std::runtime_error("YAML file missing 'cameras' section");
  }

  const auto& cameras = config_yaml["cameras"];
  for (const auto& camera_node : cameras) {
    std::string camera_name = camera_node.first.as<std::string>();
    config.cameras.emplace_back(ParseCamera(camera_name, camera_node.second));
  }

  return config;
}

}  // namespace kalibr2::ros
