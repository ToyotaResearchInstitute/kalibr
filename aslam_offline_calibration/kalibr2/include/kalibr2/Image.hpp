#pragma once

#include <optional>

#include <aslam/Time.hpp>
#include <opencv2/core.hpp>

#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>

namespace kalibr2 {

struct Image {
  aslam::Time timestamp;
  cv::Mat image;
};

std::optional<aslam::cameras::GridCalibrationTargetObservation> ToObservation(const Image& image, const aslam::cameras::GridDetector& detector);

}
