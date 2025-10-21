#include "kalibr2/Image.hpp"

#include <chrono>
#include <iostream>

namespace kalibr2 {

std::optional<aslam::cameras::GridCalibrationTargetObservation> ToObservation(
    const Image& image, const aslam::cameras::GridDetector& detector) {
  auto t0 = std::chrono::steady_clock::now();
  auto observation = aslam::cameras::GridCalibrationTargetObservation(detector.target());
  auto t1 = std::chrono::steady_clock::now();
  auto dur_construct_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

  auto t2 = std::chrono::steady_clock::now();
  bool success = detector.findTargetNoTransformation(image.image, image.timestamp, observation);
  auto t3 = std::chrono::steady_clock::now();
  auto dur_find_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();

  auto t4 = std::chrono::steady_clock::now();
  // Delete Image copy
  observation.clearImage();
  auto t5 = std::chrono::steady_clock::now();
  auto dur_clear_us = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

  std::cerr << "ToObservation timings (us): construct=" << dur_construct_us << " find=" << dur_find_us
            << " clear=" << dur_clear_us << std::endl;

  return success ? std::make_optional(observation) : std::nullopt;
}

}  // namespace kalibr2
