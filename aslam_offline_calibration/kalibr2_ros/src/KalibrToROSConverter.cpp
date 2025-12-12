#include <kalibr2_ros/KalibrToROSConverter.hpp>

namespace kalibr2::ros {

std::string ToROSDistortionModel(const std::string& kalibr_model) {
  auto it = kKalibrToROSDistortionModel.find(kalibr_model);
  if (it != kKalibrToROSDistortionModel.end()) {
    return it->second;
  }

  // Default fallback for unknown models
  return "unknown";
}

geometry_msgs::msg::TransformStamped TransformationToROS(const sm::kinematics::Transformation& transformation,
                                                         std::pair<std::string, std::string> frames) {
  geometry_msgs::msg::TransformStamped tf_msg;

  // Set frame IDs
  tf_msg.header.frame_id = frames.first;
  tf_msg.child_frame_id = frames.second;

  // Set translation
  const auto& t = transformation.t();
  tf_msg.transform.translation.x = t.x();
  tf_msg.transform.translation.y = t.y();
  tf_msg.transform.translation.z = t.z();

  // Set rotation (quaternion)
  const auto& q = transformation.q();
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  return tf_msg;
}

tf2_msgs::msg::TFMessage TransformationsToTFMessage(const std::vector<sm::kinematics::Transformation>& transformations,
                                                    const std::vector<std::pair<std::string, std::string>>& frames) {
  if (transformations.size() != frames.size()) {
    throw std::runtime_error("TransformationsToTFMessage: size mismatch between transformations and frame names");
  }

  tf2_msgs::msg::TFMessage tf_message;
  tf_message.transforms.reserve(transformations.size());

  for (size_t i = 0; i < transformations.size(); ++i) {
    tf_message.transforms.push_back(TransformationToROS(transformations[i], frames[i]));
  }

  return tf_message;
}

}  // namespace kalibr2::ros
