#pragma once

#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <kalibr2/CameraModels.hpp>

namespace kalibr2 {

namespace tools {

bool CalibrateInstrinsics(
    const std::vector<aslam::cameras::GridCalibrationTargetObservation>&
        observations,
    const boost::shared_ptr<kalibr2::models::DistortedPinhole::Geometry>&
        geometry,
    const aslam::cameras::GridDetector& detector);

}  // namespace tools

}  // namespace kalibr2
