
#include <kalibr2/CameraCalibrator.hpp>

namespace kalibr2 {

// template <typename CameraT>
// CameraCalibrator<CameraT>::CameraCalibrator() : camera_geometry_(boost::make_shared<CameraT>()) {
//     design_variable_ = typename CameraT::DesignVariable(camera_geometry_);
// }

// template <typename CameraT>
// void
// CameraCalibrator<CameraT>::AddIntrinsicDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem>
// problem) {
//     design_variable_.setActive(true, true, false);
//     problem->addDesignVariable(design_variable_.projectionDesignVariable());
//     problem->addDesignVariable(design_variable_.distortionDesignVariable());
//     problem->addDesignVariable(design_variable_.shutterDesignVariable());
// }

template <typename CameraT>
void CameraCalibrator<CameraT>::AddReprojectionErrorsForView(
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
    const aslam::cameras::GridCalibrationTargetObservation& observation,
    const aslam::backend::TransformationExpression& T_cam_w,
    const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target, const Eigen::Matrix2d& invR) {
  for (size_t i = 0; i < target->size(); ++i) {
    auto p_target = aslam::backend::HomogeneousExpression(sm::kinematics::toHomogeneous(target->point(i)));
    Eigen::Vector2d y;
    bool valid = observation.imagePoint(i, y);
    if (valid) {
      auto rerr =
          boost::make_shared<typename CameraT::ReprojectionError>(y, invR, T_cam_w * p_target, design_variable_);
      problem->addErrorTerm(rerr);
    }
  }
}

template <typename CameraT>
boost::shared_ptr<aslam::cameras::CameraGeometryBase> CameraCalibrator<CameraT>::camera_geometry() const {
  return camera_geometry_;
}

}  // namespace kalibr2
