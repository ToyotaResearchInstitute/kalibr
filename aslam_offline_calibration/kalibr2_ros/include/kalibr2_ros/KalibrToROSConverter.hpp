#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace kalibr2::ros {

// Map Kalibr2 camera model names to ROS CameraInfo distortion model names.
//
// Note: ROS CameraInfo is designed primarily for pinhole camera models.
// Non-pinhole models (omni, eucm, ds) cannot be fully represented in the
// standard CameraInfo message format, but we map them for compatibility.
//
// Standard ROS distortion models:
// - "plumb_bob": Radial-tangential distortion (Brown-Conrady model)
// - "equidistant": Fisheye/equidistant distortion model
// - "fov": Field-of-view distortion model
//
// Kalibr2 model naming: <projection>-<distortion>[-<shutter>]
// Examples: "pinhole-radtan", "omni-radtan", "pinhole-equi"
static const std::unordered_map<std::string, std::string> kKalibrToROSDistortionModel = {
    // Pinhole projection models
    {"pinhole-radtan", "plumb_bob"},     // Pinhole with radial-tangential distortion
    {"pinhole-rs-radtan", "plumb_bob"},  // Pinhole with rolling shutter
    {"pinhole-equi", "equidistant"},     // Pinhole with equidistant distortion
    {"pinhole-rs-equi", "equidistant"},  // Pinhole with equidistant + rolling shutter
    {"pinhole-fov", "fov"},              // Pinhole with FOV distortion

    // Omni projection models (limited ROS support - projection not representable)
    {"omni-radtan", "plumb_bob"},     // Omni with radial-tangential distortion
    {"omni-rs-radtan", "plumb_bob"},  // Omni with rolling shutter
    {"omni-none", ""},                // Omni without distortion

    // Extended unified camera model (not supported in standard ROS CameraInfo)
    {"eucm-none", ""},

    // Double sphere model (not supported in standard ROS CameraInfo)
    {"ds-none", ""},
};

/// @brief Convert a Kalibr2 camera model name to a ROS CameraInfo distortion model name
/// @param kalibr_model The Kalibr2 model string (e.g., "pinhole-radtan", "omni-radtan")
/// @return The corresponding ROS distortion model name, or "unknown" as fallback
std::string ToROSDistortionModel(const std::string& kalibr_model);

/// @brief Convert sm::kinematics::Transformation to geometry_msgs::msg::TransformStamped
/// @param transformation The sm::kinematics::Transformation to convert
/// @param frames A pair of strings representing (parent_frame, child_frame)
/// @return TransformStamped message with the transformation
geometry_msgs::msg::TransformStamped TransformationToROS(const sm::kinematics::Transformation& transformation,
                                                         std::pair<std::string, std::string> frames);

/// @brief Convert multiple sm::kinematics::Transformations to tf2_msgs::msg::TFMessage
/// @param transformations Vector of transformations to convert
/// @param frames Vector of pairs of strings representing (parent_frame, child_frame) for each transformation
/// @return TFMessage containing all transforms
tf2_msgs::msg::TFMessage TransformationsToTFMessage(const std::vector<sm::kinematics::Transformation>& transformations,
                                                    const std::vector<std::pair<std::string, std::string>>& frames);

}  // namespace kalibr2::ros
