#pragma once

#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace kalibr2 {

class CameraCalibratorBase {
 public:
  virtual ~CameraCalibratorBase() {}
  virtual void AddIntrinsicDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) = 0;
  virtual void AddReprojectionErrorsForView(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
                                            const aslam::cameras::GridCalibrationTargetObservation& observation,
                                            const aslam::backend::TransformationExpression& T_cam_w,
                                            const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target,
                                            const Eigen::Matrix2d& invR) = 0;
  virtual boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera_geometry() const = 0;
};

template <typename CameraT>
class CameraCalibrator : public CameraCalibratorBase {
 public:
  CameraCalibrator()
      : camera_geometry_(boost::make_shared<typename CameraT::Geometry>()), design_variable_(camera_geometry_) {}

  ~CameraCalibrator() override = default;

  void AddIntrinsicDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) {
    design_variable_.setActive(true, true, false);
    problem->addDesignVariable(design_variable_.projectionDesignVariable());
    problem->addDesignVariable(design_variable_.distortionDesignVariable());
    problem->addDesignVariable(design_variable_.shutterDesignVariable());
  }

  void AddReprojectionErrorsForView(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
                                    const aslam::cameras::GridCalibrationTargetObservation& observation,
                                    const aslam::backend::TransformationExpression& T_cam_w,
                                    const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target,
                                    const Eigen::Matrix2d& invR) {
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

  boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera_geometry() const { return camera_geometry_; }

 private:
  // Returning the designvariable and then passing it to another function makes the data types
  // leak outside of the class scope. To avoid that we keep the design variable as a member variable.
  boost::shared_ptr<typename CameraT::Geometry> camera_geometry_;
  typename CameraT::DesignVariable design_variable_;
};

}  // namespace kalibr2
