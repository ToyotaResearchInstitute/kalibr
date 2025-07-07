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


class ImageReader {
  public:
  virtual Image ReadNext() = 0;
  virtual bool HasNext() const = 0;
};


/// Transform an Image into a GridCalibrationTargetObservation using the provided GridDetector.
std::optional<aslam::cameras::GridCalibrationTargetObservation> ToObservation(const Image& image, const aslam::cameras::GridDetector& detector);

}
