#pragma once

#include <numeric>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/HomogeneousPoint.hpp>
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
#include <kalibr2/BasicMathUtils.hpp>
#include <kalibr2/CalibrationTools.hpp>
#include <kalibr2/CameraCalibrator.hpp>
#include <kalibr2/CameraModels.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/transformations.hpp>

namespace kalibr2 {

using SyncedSet = std::vector<std::optional<aslam::cameras::GridCalibrationTargetObservation>>;

namespace tools {

boost::shared_ptr<aslam::backend::TransformationBasic> AddPoseDesignVariable(
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem, const sm::kinematics::Transformation& transform,
    bool active, size_t group_id) {
  // Create design variables for the rotation and translation
  auto q_Dv = boost::make_shared<aslam::backend::RotationQuaternion>(transform.q());
  q_Dv->setActive(active);
  problem->addDesignVariable(q_Dv, group_id);

  auto t_Dv = boost::make_shared<aslam::backend::EuclideanPoint>(transform.t());
  t_Dv->setActive(active);
  problem->addDesignVariable(t_Dv, group_id);

  return boost::make_shared<aslam::backend::TransformationBasic>(q_Dv->toExpression(), t_Dv->toExpression());
}

boost::shared_ptr<aslam::backend::TransformationBasic> AddPoseDesignVariable(
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
    const sm::kinematics::Transformation& transform) {
  return AddPoseDesignVariable(problem, transform, true, 0);
}

/**
 * Create a default optimizer with the default settings used in the original
 * Kalibr code.
 */
aslam::backend::Optimizer2 CreateDefaultOptimizer() {
  auto options = aslam::backend::Optimizer2Options();
  options.nThreads = 4;
  options.convergenceDeltaX = 1e-3;
  options.convergenceDeltaJ = 1;
  options.maxIterations = 200;
  options.trustRegionPolicy = boost::make_shared<aslam::backend::LevenbergMarquardtTrustRegionPolicy>(10);

  return aslam::backend::Optimizer2(options);
}

/**
 * @brief Calibrates the intrinsic parameters of a single camera using bundle adjustment.
 *
 * This function performs a full bundle adjustment to find the optimal intrinsic parameters
 * (projection and distortion) for a given camera model. It takes a series of observations
 * of a known calibration target from different viewpoints.
 *
 * The optimization problem simultaneously refines:
 * 1. The camera's intrinsic parameters (focal length, principal point, distortion coefficients).
 * 2. The 6-DOF pose of the calibration target for each individual observation.
 *
 * @note This function modifies the input `geometry` object in-place with the optimized results.
 *
 * @tparam CameraT The specific camera model class (e.g., kalibr2::models::DistortedPinhole)
 * which defines the projection, distortion, and their design variables.
 * @param[in] observations A vector of observations of the calibration target. Each observation
 * corresponds to a single image/view.
 * @param[in,out] calibrator The camera calibrator instance that will be used for optimization.
 * @param[in] target A shared pointer to the GridCalibrationTarget object, defining the 3D structure
 * of the calibration pattern.
 * @param[in] fallback_focal_length An optional fallback focal length to use if the camera intrinsics
 * cannot be initialized from the observations.
 * @return true if the calibration and optimization were successful.
 * @return false if the optimization failed to converge or encountered a numerical issue.
 */
inline bool CalibrateSingleCamera(const std::vector<aslam::cameras::GridCalibrationTargetObservation>& observations,
                                  const boost::shared_ptr<kalibr2::CameraCalibratorBase>& camera_calibrator,
                                  const aslam::cameras::GridCalibrationTargetBase::Ptr& target,
                                  std::optional<double> fallback_focal_length) {
  // Get an initial guess for the camera focal lenght, if it fails
  // to initialize it from the observations it will fallback to the
  // fallback_focal_length argument
  bool success = camera_calibrator->camera_geometry()->initializeIntrinsics(observations, fallback_focal_length);

  auto problem = boost::make_shared<aslam::calibration::OptimizationProblem>();
  camera_calibrator->AddIntrinsicDesignVariables(problem);

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
    bool success = camera_calibrator->camera_geometry()->estimateTransformation(observation, T_t_c);
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
    camera_calibrator->AddReprojectionErrorsForView(problem, observation, T_cam_w, target, invR);
  }

  // The options of the optimizer
  // TODO(frneer): Consider exposing this to the user so we can tune it for each
  // Specific calibration if needed.
  auto optimizer = CreateDefaultOptimizer();
  optimizer.setProblem(problem);

  auto retval = optimizer.optimize();
  return !retval.linearSolverFailure;
}

/**
 * @brief Wrapper for CalibrateSingleCamera without fallback focal length.
 */
inline bool CalibrateSingleCamera(const std::vector<aslam::cameras::GridCalibrationTargetObservation>& observations,
                                  const boost::shared_ptr<kalibr2::CameraCalibratorBase>& calibrator,
                                  const aslam::cameras::GridCalibrationTargetBase::Ptr& target) {
  return CalibrateSingleCamera(observations, calibrator, target, std::nullopt);
}

/**
 * @brief Calibrates a stereo camera pair, optimizing both intrinsics and extrinsics jointly.
 *
 * This function performs a joint bundle adjustment to fully calibrate a stereo camera pair.
 * It uses synchronized observations of a calibration target to robustly estimate all parameters.
 * An initial guess for the stereo baseline is found by taking the median of PnP solutions from
 * all views where the target is visible in both cameras.
 *
 * The large-scale optimization problem simultaneously refines:
 * 1. Intrinsic parameters for the left camera.
 * 2. Intrinsic parameters for the right camera.
 * 3. The 6-DOF rigid body transformation (baseline) from the left camera to the right camera.
 * 4. The 6-DOF pose of the calibration target for each time step.
 *
 * @note This function modifies the input geometry objects (`geometry_camera_L`, `geometry_camera_H`)
 * in-place with the optimized intrinsic parameters.
 *
 * @tparam CameraLT The camera model class for the Left camera.
 * @tparam CameraHT The camera model class for the Right camera.
 * @param[in,out] geometry_camera_L The geometry model for the left camera. Will be updated with results.
 * @param[in,out] geometry_camera_H The geometry model for the right camera. Will be updated with results.
 * @param[in] observations_camera_L A vector of time-synchronized optional observations for the left camera.
 * @param[in] observations_camera_H A vector of time-synchronized optional observations for the right camera.
 * @param[in] target The shared definition of the calibration target.
 * @return The optimized 6-DOF transformation from the left camera to the right camera ($T_{camH\_camL}$).
 * @throws std::runtime_error if the number of observations for the two cameras do not match, or if
 * the optimization's linear solver fails.
 */
inline sm::kinematics::Transformation CalibrateStereoPair(
    boost::shared_ptr<kalibr2::CameraCalibratorBase> calibrator_L,
    boost::shared_ptr<kalibr2::CameraCalibratorBase> calibrator_H,
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

    auto success = calibrator_L->camera_geometry()->estimateTransformation(observations_camera_L[i].value(), T_L);
    if (!success) {
      SM_ERROR_STREAM("Failed to estimate transformation for camera L at index " << i);
      continue;
    }
    success = calibrator_H->camera_geometry()->estimateTransformation(observations_camera_H[i].value(), T_H);
    if (!success) {
      SM_ERROR_STREAM("Failed to estimate transformation for camera H at index " << i);
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
  auto median_translation = kalibr2::math::median(translations);

  std::vector<Eigen::Vector3d> rotation_parameters;
  std::transform(transformations.begin(), transformations.end(), std::back_inserter(rotation_parameters),
                 [](const sm::kinematics::Transformation& t) {
                   return sm::kinematics::RotationVector().rotationMatrixToParameters(t.C());
                 });
  auto median_rotation_parameters = kalibr2::math::median(rotation_parameters);
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
      calibrator_L->camera_geometry()->estimateTransformation(observations_camera_L[i].value(), T_L);
    } else if (observations_camera_H[i].has_value()) {
      calibrator_H->camera_geometry()->estimateTransformation(observations_camera_H[i].value(), T_H);
      T_L = T_H * T_H_L_baseline.inverse();
    } else {
      continue;  // Skip if neither observation is available
    }
    auto target_pose_dv = kalibr2::tools::AddPoseDesignVariable(problem, T_L);
    target_pose_dvs.push_back(target_pose_dv);
  }

  // Add the intrinsic design variables for both cameras
  calibrator_L->AddIntrinsicDesignVariables(problem);
  calibrator_H->AddIntrinsicDesignVariables(problem);

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
    calibrator_L->AddReprojectionErrorsForView(problem, observations_camera_L[i].value(), T_cam_w, target, invR);
  }

  // For camera H
  for (size_t i = 0; i < observations_camera_H.size(); ++i) {
    if (!observations_camera_H[i].has_value()) {
      continue;  // Skip if observation is not available
    }
    auto T_cam_w = baseline_dv->toExpression() * target_pose_dvs[i]->toExpression().inverse();
    calibrator_H->AddReprojectionErrorsForView(problem, observations_camera_H[i].value(), T_cam_w, target, invR);
  }

  // The options of the optimizer
  auto optimizer = CreateDefaultOptimizer();
  optimizer.setProblem(problem);

  auto retval = optimizer.optimize();
  if (retval.linearSolverFailure) {
    std::runtime_error("Linear solver failed during optimization.");
  }

  // Update the transformation with the optimized values
  auto baseline_HL = sm::kinematics::Transformation(baseline_dv->toExpression().toTransformationMatrix());

  return baseline_HL;
}

/**
 * @brief Estimates the pose of the calibration target in the reference camera frame.
 *
 * This function selects the camera observation with the highest number of detected corners
 * from a synchronized set of observations, estimates the transformation from the target to
 * that camera, and then composes it with the baseline transformations to obtain the pose
 * of the target in the reference (first) camera frame.
 *
 * @param calibrators Vector of shared pointers to camera calibrator objects, one for each camera.
 * @param synced_set Synchronized set of optional grid calibration target observations, one per camera.
 * @param baseline_guesses Vector of baseline transformations between consecutive cameras.
 * @return sm::kinematics::Transformation The estimated transformation from the target to the reference camera frame.
 */
sm::kinematics::Transformation getTargetPoseGuess(
    const std::vector<boost::shared_ptr<kalibr2::CameraCalibratorBase>>& calibrators, const SyncedSet& synced_set,
    const std::vector<sm::kinematics::Transformation>& baseline_guesses) {
  std::vector<size_t> n_corners;
  std::transform(synced_set.begin(), synced_set.end(), std::back_inserter(n_corners),
                 [](const std::optional<aslam::cameras::GridCalibrationTargetObservation>& obs) {
                   if (!obs.has_value()) {
                     return 0U;
                   }
                   std::vector<unsigned int> corners_idx;
                   auto n_corners = obs.value().getCornersIdx(corners_idx);
                   return n_corners;
                 });
  auto max_index = std::distance(n_corners.begin(), std::max_element(n_corners.begin(), n_corners.end()));
  auto geometry = calibrators[max_index]->camera_geometry();
  auto observation = synced_set[max_index];

  auto T_t_cN = sm::kinematics::Transformation();
  geometry->estimateTransformation(observation.value(), T_t_cN);

  auto T_t_c0 = std::accumulate(baseline_guesses.begin(), baseline_guesses.begin() + max_index, T_t_cN,
                                std::multiplies<sm::kinematics::Transformation>());

  return T_t_c0;
}

// This function reproduces what solveFullBatch() does in the original python Kalibr code.
/**
 * @brief Calibrates a multi-camera rig using synchronized observations and initial baseline guesses.
 *
 * This function sets up and solves an optimization problem to calibrate the extrinsic transformations (baselines)
 * between multiple cameras in a rig. It uses a set of camera calibrators, synchronized observation sets, a calibration
 * target, and initial guesses for the baselines. The function adds intrinsic and extrinsic design variables, builds
 * reprojection error terms for each observation, and optimizes the problem to refine the baseline transformations.
 *
 * @param calibrators Vector of shared pointers to camera calibrator objects, one for each camera in the rig.
 * @param synced_observations Vector of synchronized observation sets, where each set contains optional observations
 *        for each camera at a given time.
 * @param target Shared pointer to the calibration target used for reprojection error computation.
 * @param baseline_guesses Vector of initial guesses for the baseline transformations between cameras.
 * @return std::vector<sm::kinematics::Transformation> The optimized baseline transformations between cameras.
 *
 * @throws std::runtime_error If the optimizer's linear solver fails during optimization.
 */
std::vector<sm::kinematics::Transformation> CalibrateMultiCameraRig(
    const std::vector<boost::shared_ptr<kalibr2::CameraCalibratorBase>>& calibrators,
    const std::vector<SyncedSet>& synced_observations, const aslam::cameras::GridCalibrationTargetBase::Ptr& target,
    const std::vector<sm::kinematics::Transformation>& baseline_guesses) {
  auto problem = boost::make_shared<aslam::calibration::OptimizationProblem>();
  for (const auto& calibrator : calibrators) {
    calibrator->AddIntrinsicDesignVariables(problem);
  }

  auto baseline_dvs = std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>>();
  std::transform(baseline_guesses.begin(), baseline_guesses.end(), std::back_inserter(baseline_dvs),
                 [&problem](const sm::kinematics::Transformation& t) {
                   return kalibr2::tools::AddPoseDesignVariable(problem, t);
                 });

  constexpr double corner_uncertainty = 1.0;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * corner_uncertainty * corner_uncertainty;
  Eigen::Matrix2d invR = R.inverse();

  std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>> target_pose_dvs;
  for (const auto& synced_set : synced_observations) {
    auto T0 = getTargetPoseGuess(calibrators, synced_set, baseline_guesses);
    auto target_pose_dv = kalibr2::tools::AddPoseDesignVariable(problem, T0);
    target_pose_dvs.push_back(target_pose_dv);
    for (size_t i = 0; i < synced_set.size(); ++i) {
      if (!synced_set[i].has_value()) {
        continue;  // Skip if observation is not available
      }
      // Build pose chain (target->cam0->baselines->camN)
      auto T_cam_w = target_pose_dv->toExpression().inverse();
      for (size_t j = 0; j < i; ++j) {
        T_cam_w = baseline_dvs[j]->toExpression() * T_cam_w;
      }
      // Add error terms
      calibrators[i]->AddReprojectionErrorsForView(problem, synced_set[i].value(), T_cam_w, target, invR);
    }
  }

  auto optimizer = CreateDefaultOptimizer();
  optimizer.setProblem(problem);

  auto retval = optimizer.optimize();
  if (retval.linearSolverFailure) {
    throw std::runtime_error("Linear solver failed during optimization.");
  }

  std::vector<sm::kinematics::Transformation> baselines;
  std::transform(baseline_dvs.begin(), baseline_dvs.end(), std::back_inserter(baselines),
                 [](const boost::shared_ptr<aslam::backend::TransformationBasic>& dv) {
                   return sm::kinematics::Transformation(dv->toExpression().toTransformationMatrix());
                 });
  return baselines;
}

// This replicates the logic of the constructor of CalibrationTarget in the original Kalibr code.
std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>> getLandmarkDesignVariables(
    boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
    const aslam::cameras::GridCalibrationTargetBase::Ptr& target) {
  // generate a index vector from 0 to target->size() - 1
  std::vector<size_t> indices(target->size());
  std::iota(indices.begin(), indices.end(), 0);
  std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>> landmark_dvs;
  std::transform(indices.begin(), indices.end(), std::back_inserter(landmark_dvs), [&target](size_t i) {
    // This line is failing because target->point(i) returns a Vector3d
    // and not a Vector4d as expected by the HomogeneousPoint constructor.
    return boost::make_shared<aslam::backend::HomogeneousPoint>(sm::kinematics::toHomogeneous(target->point(i)));
  });
  return landmark_dvs;
}

struct BatchProblemStruct {
  boost::shared_ptr<aslam::calibration::OptimizationProblem> problem;
  std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>> baseline_dvs;
  boost::shared_ptr<aslam::backend::TransformationBasic> target_pose_dv;
  std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>> landmark_dvs;
};

BatchProblemStruct CreateBatchProblem(
    const std::vector<boost::shared_ptr<kalibr2::CameraCalibratorBase>>& camera_calibrators,
    const SyncedSet& synced_set, const sm::kinematics::Transformation& T_tc_guess,
    const std::vector<sm::kinematics::Transformation>& baseline_guesses,
    const aslam::cameras::GridCalibrationTargetBase::Ptr& target) {
  // TODO(frneer): Some methods, like AddPoseDesignVariable expect a shared_ptr.
  // But passing shared_ptr around prevents copy elision. So... can we user refs?
  auto problem = boost::make_shared<aslam::calibration::OptimizationProblem>();
  constexpr int TRANSFORMATION_GROUP_ID = 0;
  constexpr int CALIBRATION_GROUP_ID = 1;
  constexpr int LANDMARK_GROUP_ID = 2;
  // Maybe T_tc_guess should be computed inside this same function, check if it's used outside.

  // 1. Add target pose design variable
  auto T_tc_guess_dv = AddPoseDesignVariable(problem, T_tc_guess, false, TRANSFORMATION_GROUP_ID);

  // 2. Add all baseline and target pose design variables
  std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>> baseline_dvs;
  for (const auto& baseline : baseline_guesses) {
    baseline_dvs.push_back(AddPoseDesignVariable(problem, baseline, true, CALIBRATION_GROUP_ID));
  }

  // 3. Add landmark design variables
  auto landmark_dvs = getLandmarkDesignVariables(problem, target);
  for (const auto& landmark : landmark_dvs) {
    problem->addDesignVariable(landmark, LANDMARK_GROUP_ID);
  }

  // 4. Add camera intrinsic design variables
  for (const auto& calibrator : camera_calibrators) {
    calibrator->AddIntrinsicDesignVariables(problem, CALIBRATION_GROUP_ID);
  }

  // 5. Add reprojection error terms for each camera
  constexpr double corner_uncertainty = 1.0;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * corner_uncertainty * corner_uncertainty;
  Eigen::Matrix2d invR = R.inverse();

  std::vector<std::vector<boost::shared_ptr<aslam::backend::ErrorTerm>>> reprojection_errors;
  for (size_t i = 0; i < synced_set.size(); ++i) {
    if (!synced_set[i].has_value()) {
      continue;  // Skip if observation is not available
    }
    // Build pose chain (target->cam0->baselines->camN)
    // The multiplication here then uses the internals lhs to compute something and it's gone... FIx
    auto T_cam_w = T_tc_guess_dv->toExpression().inverse();
    for (size_t j = 0; j < i; ++j) {
      T_cam_w = baseline_dvs[j]->toExpression() * T_cam_w;
    }
    // Add error terms
    // Note(frneer): Original code adds an optional error term using blake-zisserman.
    // As for now we have no evidence that is an used feature.
    camera_calibrators[i]->AddAndStoreReprojectionErrorsForView(problem, synced_set[i].value(), T_cam_w, landmark_dvs,
                                                                invR);
  }
  auto batch_problem_struct = BatchProblemStruct();
  batch_problem_struct.problem = problem;
  batch_problem_struct.baseline_dvs = baseline_dvs;
  batch_problem_struct.target_pose_dv = T_tc_guess_dv;
  batch_problem_struct.landmark_dvs = landmark_dvs;
  return batch_problem_struct;
}

}  // namespace tools

}  // namespace kalibr2
