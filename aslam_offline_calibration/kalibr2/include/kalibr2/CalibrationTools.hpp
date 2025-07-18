#pragma once

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <kalibr2/CalibrationTools.hpp>
#include <kalibr2/CameraModels.hpp>

namespace kalibr2 {

namespace tools {

template <typename CameraGeometryT, typename DesignVariableT, typename ReprojectionErrorT>
bool CalibrateIntrinsics(const std::vector<aslam::cameras::GridCalibrationTargetObservation>& observations,
                         const boost::shared_ptr<CameraGeometryT>& geometry,
                         const aslam::cameras::GridDetector& detector, std::optional<double> fallback_focal_length) {
  // Get an initial guess for the camera focal lenght, if it fails
  // To initialize it from the observations it will fallback to the
  // fallback_focal_length argument
  bool success = geometry->initializeIntrinsics(observations, fallback_focal_length);

  // Setup the basic problem adding the intrinsic design variables
  // - Projection - Active
  // - Distortion - Active
  // - Shutter - NOT Active
  // Deactivated variables are added to the problem object but
  // aren't taken into consideration for optimization.
  auto problem = boost::make_shared<aslam::calibration::OptimizationProblem>();
  auto design_variable = DesignVariableT(geometry);
  design_variable.setActive(true, true, false);
  problem->addDesignVariable(design_variable.projectionDesignVariable());
  problem->addDesignVariable(design_variable.distortionDesignVariable());
  problem->addDesignVariable(design_variable.shutterDesignVariable());

  // Corner uncertainty
  double corner_uncertainty = 1.0;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * corner_uncertainty * corner_uncertainty;
  Eigen::Matrix2d invR = R.inverse();

  auto target = detector.target();

  // It's required to capture the target_poses to prevent them from being
  // destroyed before the optimizer runs. Because the optimizer doesn't handle
  // the ownership of the design variables, we need to ensure
  // they stay alive.
  std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>> target_pose_dvs;
  for (const auto& obs : observations) {
    // Estimate transformation from target to camera
    sm::kinematics::Transformation T_t_c;
    bool success = geometry->estimateTransformation(obs, T_t_c);
    if (!success) {
      SM_WARN("Failed to estimate transformation for observation.");
      continue;
    }

    // Add the extrinsic design variables for this observation.
    auto q_Dv = boost::make_shared<aslam::backend::RotationQuaternion>(T_t_c.q());
    q_Dv->setActive(true);
    problem->addDesignVariable(q_Dv);
    auto t_Dv = boost::make_shared<aslam::backend::EuclideanPoint>(T_t_c.t());
    t_Dv->setActive(true);
    problem->addDesignVariable(t_Dv);
    // Preserve target_pose_dv
    auto target_pose_dv =
        boost::make_shared<aslam::backend::TransformationBasic>(q_Dv->toExpression(), t_Dv->toExpression());
    target_pose_dvs.push_back(target_pose_dv);

    auto T_cam_w = target_pose_dv->toExpression().inverse();

    // Add error terms for each point in the target.
    // To optimize taking into account all visible corners
    // In each observation
    for (size_t i = 0; i < target->size(); ++i) {
      // Solve the p2p problem and build the reprojection error.
      // Adding the later as error term to the problem.
      auto p_target = aslam::backend::HomogeneousExpression(sm::kinematics::toHomogeneous(target->point(i)));
      Eigen::Vector2d y;
      bool valid = obs.imagePoint(i, y);
      if (valid) {
        auto rerr = boost::make_shared<ReprojectionErrorT>(y, invR, T_cam_w * p_target, design_variable);
        problem->addErrorTerm(rerr);
      }
    }
  }

  // The options of the optimizer
  // TODO(frneer): Consider exposing this to the user so we can tune it for each
  // Specific calibration if needed.
  auto options = aslam::backend::Optimizer2Options();
  options.nThreads = 4;
  options.convergenceDeltaX = 1e-3;
  options.convergenceDeltaJ = 1;
  options.maxIterations = 200;
  options.trustRegionPolicy = boost::make_shared<aslam::backend::LevenbergMarquardtTrustRegionPolicy>(10);

  auto optimizer = aslam::backend::Optimizer2(options);
  optimizer.setProblem(problem);

  auto retval = optimizer.optimize();
  return !retval.linearSolverFailure;
}

template <typename CameraGeometryT, typename DesignVariableT, typename ReprojectionErrorT>
bool CalibrateIntrinsics(const std::vector<aslam::cameras::GridCalibrationTargetObservation>& observations,
                         const boost::shared_ptr<CameraGeometryT>& geometry,
                         const aslam::cameras::GridDetector& detector) {
  return CalibrateIntrinsics<CameraGeometryT, DesignVariableT, ReprojectionErrorT>(observations, geometry, detector,
                                                                                   std::nullopt);
}

}  // namespace tools

}  // namespace kalibr2
