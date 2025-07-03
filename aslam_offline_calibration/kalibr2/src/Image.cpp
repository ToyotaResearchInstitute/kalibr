#include "kalibr2/Image.hpp"

std::optional<aslam::cameras::GridCalibrationTargetObservation> ToObservation(const Image& image, const aslam::cameras::GridDetector& detector) {
  auto observation = aslam::cameras::GridCalibrationTargetObservation(detector.target());
  bool success = detector.findTargetNoTransformation(image.image, image.timestamp, observation);

  // Delete Image copy
  observation.clearImage();

  return success ? std::make_optional(observation) : std::nullopt;
}
