#pragma once

#include <optional>

#include <aslam/Time.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <opencv2/core.hpp>

namespace kalibr2 {

struct Image {
  aslam::Time timestamp;
  cv::Mat image;
};

/// Transform an Image into a GridCalibrationTargetObservation using the
/// provided GridDetector.
/* Aims to translate extractCornersFromDataset python function
https://github.com/ToyotaResearchInstitute/kalibr/blob/c9433f9b4bbf9d99f890c480e29a34f04a3c6af8/aslam_offline_calibration/kalibr/python/kalibr_common/TargetExtractor.py#L34
To C++. With some simplifications:
  1. No support for parallel processing of the images (dropped because for the
moment we can only read images sequentially)
  2. We don't use the findTarget with transformation, since it's not being used
in kalibr_calibrate_cameras code which is the part we currently care about.
  3. We always clear the image in the observation to save space, no reason to
keep it around.
*/
std::optional<aslam::cameras::GridCalibrationTargetObservation> ToObservation(
    const Image& image, const aslam::cameras::GridDetector& detector);

}  // namespace kalibr2
