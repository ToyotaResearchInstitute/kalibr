#pragma once

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <kalibr2/CalibrationTools.hpp>
#include <kalibr2/CameraModels.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/transformations.hpp>

namespace kalibr2 {

using SyncedSet = std::vector<std::optional<aslam::cameras::GridCalibrationTargetObservation>>;

namespace tools {

boost::shared_ptr<aslam::backend::TransformationBasic> AddPoseDesignVariable(
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
    const sm::kinematics::Transformation& transform) {
  // Create design variables for the rotation and translation
  auto q_Dv = boost::make_shared<aslam::backend::RotationQuaternion>(transform.q());
  q_Dv->setActive(true);
  problem->addDesignVariable(q_Dv);

  auto t_Dv = boost::make_shared<aslam::backend::EuclideanPoint>(transform.t());
  t_Dv->setActive(true);
  problem->addDesignVariable(t_Dv);

  // Create the transformation expression node
  return boost::make_shared<aslam::backend::TransformationBasic>(q_Dv->toExpression(), t_Dv->toExpression());
}

template <typename CameraT>
typename CameraT::DesignVariable AddIntrinsicDesignVariables(
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
    const boost::shared_ptr<aslam::cameras::CameraGeometryBase>& geometry) {
  auto design_variable = typename CameraT::DesignVariable(geometry);
  design_variable.setActive(true, true, false);
  problem->addDesignVariable(design_variable.projectionDesignVariable());
  problem->addDesignVariable(design_variable.distortionDesignVariable());
  problem->addDesignVariable(design_variable.shutterDesignVariable());
  return design_variable;
}

template <typename CameraT>
void AddReprojectionErrorsForView(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
                                  const aslam::cameras::GridCalibrationTargetObservation observation,
                                  const aslam::backend::TransformationExpression& T_cam_w,
                                  const typename CameraT::DesignVariable& design_variable,
                                  const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target,
                                  const Eigen::Matrix2d& invR) {
  for (size_t i = 0; i < target->size(); ++i) {
    // Solve the p2p problem and build the reprojection error.
    // And add it as error term to the problem.
    auto p_target = aslam::backend::HomogeneousExpression(sm::kinematics::toHomogeneous(target->point(i)));
    Eigen::Vector2d y;
    bool valid = observation.imagePoint(i, y);
    if (valid) {
      auto rerr = boost::make_shared<typename CameraT::ReprojectionError>(y, invR, T_cam_w * p_target, design_variable);
      problem->addErrorTerm(rerr);
    }
  }
}

aslam::backend::Optimizer2 CreateDefaultOptimizer() {
  auto options = aslam::backend::Optimizer2Options();
  options.nThreads = 4;
  options.convergenceDeltaX = 1e-3;
  options.convergenceDeltaJ = 1;
  options.maxIterations = 200;
  options.trustRegionPolicy = boost::make_shared<aslam::backend::LevenbergMarquardtTrustRegionPolicy>(10);

  return aslam::backend::Optimizer2(options);
}

template <typename CameraT>
bool CalibrateIntrinsics(const std::vector<aslam::cameras::GridCalibrationTargetObservation>& observations,
                         const boost::shared_ptr<aslam::cameras::CameraGeometryBase>& geometry,
                         const aslam::cameras::GridCalibrationTargetBase::Ptr& target,
                         std::optional<double> fallback_focal_length) {
  // Get an initial guess for the camera focal lenght, if it fails
  // to initialize it from the observations it will fallback to the
  // fallback_focal_length argument
  bool success = geometry->initializeIntrinsics(observations, fallback_focal_length);

  // Setup the basic problem adding the intrinsic design variables
  // - Projection - Active
  // - Distortion - Active
  // - Shutter - NOT Active
  // Deactivated variables are added to the problem object but
  // aren't taken into consideration for optimization.
  auto problem = boost::make_shared<aslam::calibration::OptimizationProblem>();
  auto design_variable = AddIntrinsicDesignVariables<CameraT>(problem, geometry);

  // Corner uncertainty
  constexpr double corner_uncertainty = 1.0;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * corner_uncertainty * corner_uncertainty;
  Eigen::Matrix2d invR = R.inverse();

  // It's required to capture the target_poses to prevent them from being
  // destroyed before the optimizer runs. Because the optimizer doesn't handle
  // the ownership of the design variables, we need to ensure
  // they stay alive.
  std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>> target_pose_dvs;
  for (const auto& observation : observations) {
    // Estimate transformation from target to camera
    sm::kinematics::Transformation T_t_c;
    bool success = geometry->estimateTransformation(observation, T_t_c);
    if (!success) {
      SM_WARN("Failed to estimate transformation for observation.");
      continue;
    }

    // Add the extrinsic design variables for this observation.
    auto target_pose_dv = AddPoseDesignVariable(problem, T_t_c);
    target_pose_dvs.push_back(target_pose_dv);

    auto T_cam_w = target_pose_dv->toExpression().inverse();

    // Add error terms for each point in the target
    // taking into account all visible corners
    // in each observation for optimization.
    AddReprojectionErrorsForView<CameraT>(problem, observation, T_cam_w, design_variable, target, invR);
  }

  // The options of the optimizer
  // TODO(frneer): Consider exposing this to the user so we can tune it for each
  // Specific calibration if needed.
  auto optimizer = CreateDefaultOptimizer();
  optimizer.setProblem(problem);

  auto retval = optimizer.optimize();
  return !retval.linearSolverFailure;
}

template <typename CameraT>
bool CalibrateIntrinsics(const std::vector<aslam::cameras::GridCalibrationTargetObservation>& observations,
                         const boost::shared_ptr<aslam::cameras::CameraGeometryBase>& geometry,
                         const aslam::cameras::GridCalibrationTargetBase::Ptr& target) {
  return CalibrateIntrinsics<CameraT>(observations, geometry, target, std::nullopt);
}

std::vector<std::optional<aslam::cameras::GridCalibrationTargetObservation>> GetAllObservationsFromSource(
    const std::vector<kalibr2::SyncedSet>& sets, size_t source_index) {
  std::vector<std::optional<aslam::cameras::GridCalibrationTargetObservation>> observations;

  for (const auto& set : sets) {
    observations.push_back(set.at(source_index));
  }

  return observations;
}

double median(std::vector<double> vec) {
  if (vec.empty()) {
    throw std::runtime_error("Cannot compute median of an empty vector.");
  }
  std::nth_element(vec.begin(), vec.begin() + vec.size() / 2, vec.end());
  return vec.at(vec.size() / 2);
}

Eigen::Vector3d median(std::vector<Eigen::Vector3d> vec) {
  if (vec.empty()) {
    throw std::runtime_error("Cannot compute median of an empty vector.");
  }
  std::vector<double> x, y, z;
  for (const auto& v : vec) {
    x.push_back(v.x());
    y.push_back(v.y());
    z.push_back(v.z());
  }
  return Eigen::Vector3d(median(x), median(y), median(z));
}

template <typename CameraLT, typename CameraHT>
sm::kinematics::Transformation stereoCalibrate(
    boost::shared_ptr<aslam::cameras::CameraGeometryBase> geometry_camera_L,
    boost::shared_ptr<aslam::cameras::CameraGeometryBase> geometry_camera_H,
    const std::vector<std::optional<aslam::cameras::GridCalibrationTargetObservation>>& observations_camera_L,
    const std::vector<std::optional<aslam::cameras::GridCalibrationTargetObservation>>& observations_camera_H,
    const aslam::cameras::GridCalibrationTargetBase::Ptr& target) {
  if (observations_camera_L.size() != observations_camera_H.size()) {
    throw std::runtime_error("The number of observations for both cameras must be the same.");
  }
  // --- Get the initial guess for the transformation ---
  // As the median of all pnp solutions
  // Iterate throught shared observations between cameras
  sm::kinematics::Transformation T_L;
  sm::kinematics::Transformation T_H;
  std::vector<sm::kinematics::Transformation> transformations;
  for (size_t i = 0; i < observations_camera_L.size(); ++i) {
    if (!observations_camera_L[i].has_value() || !observations_camera_H[i].has_value()) {
      continue;  // Skip if either observation is not available
    }

    auto success = geometry_camera_L->estimateTransformation(observations_camera_L[i].value(), T_L);
    if (!success) {
      std::cerr << "Failed to estimate transformation for camera L at index " << i << std::endl;
      continue;
    }
    success = geometry_camera_H->estimateTransformation(observations_camera_H[i].value(), T_H);
    if (!success) {
      std::cerr << "Failed to estimate transformation for camera H at index " << i << std::endl;
      continue;
    }

    // Store the transformation from camera L to camera H
    transformations.push_back(T_H.inverse() * T_L);
  }

  std::vector<Eigen::Vector3d> translations;
  std::transform(transformations.begin(), transformations.end(), std::back_inserter(translations),
                 [](const sm::kinematics::Transformation& t) {
                   return t.t();
                 });
  auto median_translation = median(translations);

  std::vector<Eigen::Vector3d> rotation_parameters;
  std::transform(transformations.begin(), transformations.end(), std::back_inserter(rotation_parameters),
                 [](const sm::kinematics::Transformation& t) {
                   return sm::kinematics::RotationVector().rotationMatrixToParameters(t.C());
                 });
  auto median_rotation_parameters = median(rotation_parameters);
  auto median_rotation_matrix = sm::kinematics::RotationVector().parametersToRotationMatrix(median_rotation_parameters);

  sm::kinematics::Transformation T_H_L_baseline =
      sm::kinematics::rt2Transform(median_rotation_matrix, median_translation);

  auto problem = boost::make_shared<aslam::calibration::OptimizationProblem>();
  auto baseline_dv = kalibr2::tools::AddPoseDesignVariable(problem, T_H_L_baseline);

  // It's required to capture the target_poses to prevent them from being
  // destroyed before the optimizer runs. Because the optimizer doesn't handle
  // the ownership of the design variables, we need to ensure they stay alive.
  auto target_pose_dvs = std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>>();

  for (size_t i = 0; i < observations_camera_L.size(); ++i) {
    // Note: The original code (see CameraInitializers.py) doesn't check
    // if the transform estimations are successful.
    // We kept that behavior for the time being.
    if (observations_camera_L[i].has_value()) {
      geometry_camera_L->estimateTransformation(observations_camera_L[i].value(), T_L);
    } else if (observations_camera_H[i].has_value()) {
      geometry_camera_H->estimateTransformation(observations_camera_H[i].value(), T_H);
      T_L = T_H * T_H_L_baseline.inverse();
    }
    auto target_pose_dv = kalibr2::tools::AddPoseDesignVariable(problem, T_L);
    target_pose_dvs.push_back(target_pose_dv);
  }

  // Add the intrinsic design variables for both cameras
  auto design_variable_camera_L = kalibr2::tools::AddIntrinsicDesignVariables<CameraLT>(problem, geometry_camera_L);
  auto design_variable_camera_H = kalibr2::tools::AddIntrinsicDesignVariables<CameraHT>(problem, geometry_camera_H);

  // Corner uncertainty
  constexpr double corner_uncertainty = 1.0;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * corner_uncertainty * corner_uncertainty;
  Eigen::Matrix2d invR = R.inverse();

  // For camera L
  for (size_t i = 0; i < observations_camera_L.size(); ++i) {
    if (!observations_camera_L[i].has_value()) {
      continue;  // Skip if observation is not available
    }
    auto T_cam_w = target_pose_dvs[i]->toExpression().inverse();
    kalibr2::tools::AddReprojectionErrorsForView<CameraLT>(problem, observations_camera_L[i].value(), T_cam_w,
                                                           design_variable_camera_L, target, invR);
  }

  // For camera H
  for (size_t i = 0; i < observations_camera_H.size(); ++i) {
    if (!observations_camera_H[i].has_value()) {
      continue;  // Skip if observation is not available
    }
    auto T_cam_w = baseline_dv->toExpression() * target_pose_dvs[i]->toExpression().inverse();
    kalibr2::tools::AddReprojectionErrorsForView<CameraHT>(problem, observations_camera_H[i].value(), T_cam_w,
                                                           design_variable_camera_H, target, invR);
  }

  // The options of the optimizer
  // TODO(frneer): Consider exposing this to the user so we can tune it for each
  // Specific calibration if needed.
  auto optimizer = CreateDefaultOptimizer();
  optimizer.setProblem(problem);

  auto retval = optimizer.optimize();
  if (retval.linearSolverFailure) {
    std::runtime_error("Linear solver failed during optimization.");
  }

  // Note (frneer): It doesn't seems right to use baseline_dv, here since it's not the actual optimized
  // transformation from camera L to camera H....
  // Update the transformation with the optimized values
  auto baseline_HL = sm::kinematics::Transformation(baseline_dv->toExpression().toTransformationMatrix());

  return baseline_HL;
}

}  // namespace tools

}  // namespace kalibr2
