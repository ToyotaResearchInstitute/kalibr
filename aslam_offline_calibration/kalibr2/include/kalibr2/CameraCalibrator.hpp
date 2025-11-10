#pragma once

#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace kalibr2 {

/**
 * @brief Abstract base class for camera calibration.
 *
 * Provides an interface for adding intrinsic design variables and reprojection error terms
 * to an optimization problem, as well as accessing the underlying camera geometry.
 */
class CameraCalibratorBase {
 public:
  /**
   * @brief Virtual destructor.
   */
  virtual ~CameraCalibratorBase() {}

  /**
   * @brief Add intrinsic design variables to the optimization problem with a specified group ID.
   * @param problem Shared pointer to the optimization problem.
   * @param group_id The group ID for the design variables.
   */
  virtual void AddIntrinsicDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
                                           size_t group_id) = 0;

  /**
   * @brief Same as AddIntrinsicDesignVariables but with default group ID 0.
   * @param problem Shared pointer to the optimization problem.
   */
  virtual void AddIntrinsicDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) = 0;

  /**
   * @brief Add reprojection error terms for a given view to the optimization problem.
   * For all observed corners in the observation, creates reprojection error terms and adds them to the problem.
   * @param problem Shared pointer to the optimization problem.
   * @param observation The grid calibration target observation.
   * @param T_cam_w Transformation expression from world to camera.
   * @param target Shared pointer to the grid calibration target.
   * @param invR Inverse covariance matrix for the reprojection error.
   */
  virtual void AddReprojectionErrorsForView(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
                                            const aslam::cameras::GridCalibrationTargetObservation& observation,
                                            const aslam::backend::TransformationExpression& T_cam_w,
                                            const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target,
                                            const Eigen::Matrix2d& invR) const = 0;
  /**
   * @brief Same as AddReprojectionErrorsForView but stores reprojection errors in the state of this class.
   *
   * @param problem Shared pointer to the optimization problem.
   * @param observation The grid calibration target observation.
   * @param T_cam_w Transformation expression from world to camera.
   * @param target Shared pointer to the grid calibration target.
   * @param invR Inverse covariance matrix for the reprojection error.
   */
  virtual void AddAndStoreReprojectionErrorsForView(
      boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
      const aslam::cameras::GridCalibrationTargetObservation& observation,
      const aslam::backend::TransformationExpression& T_cam_w,
      // const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target,
      const std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>>& target_points,
      const Eigen::Matrix2d& invR) = 0;

  /**
   * @brief Get the underlying camera geometry.
   * @return Shared pointer to the camera geometry base.
   */
  virtual boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera_geometry() const = 0;

  virtual void PrintReprojectionErrorsMean() const = 0;
  virtual void PrintReprojectionErrorStatistics() const = 0;
};

template <typename CameraT>
class CameraCalibrator : public CameraCalibratorBase {
 public:
  CameraCalibrator()
      : camera_geometry_(boost::make_shared<typename CameraT::Geometry>()), design_variable_(camera_geometry_) {}

  ~CameraCalibrator() override = default;

  void AddIntrinsicDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
                                   size_t group_id) {
    design_variable_.setActive(true, true, false);
    problem->addDesignVariable(design_variable_.projectionDesignVariable(), group_id);
    problem->addDesignVariable(design_variable_.distortionDesignVariable(), group_id);
    problem->addDesignVariable(design_variable_.shutterDesignVariable(), group_id);
  }

  void AddIntrinsicDesignVariables(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem) {
    AddIntrinsicDesignVariables(problem, 0);
  }

  void AddReprojectionErrorsForView(boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
                                    const aslam::cameras::GridCalibrationTargetObservation& observation,
                                    const aslam::backend::TransformationExpression& T_cam_w,
                                    const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target,
                                    const Eigen::Matrix2d& invR) const {
    // std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>> target_points;
    // for (size_t i = 0; i < target->size(); ++i) {
    //   target_points.push_back(boost::make_shared<aslam::backend::HomogeneousPoint>(sm::kinematics::toHomogeneous(target->point(i))));
    // }
    AddReprojectionErrorsForViewImplementation(problem, observation, T_cam_w, target, invR, std::nullopt);
  }

  void AddAndStoreReprojectionErrorsForView(
      boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
      const aslam::cameras::GridCalibrationTargetObservation& observation,
      const aslam::backend::TransformationExpression& T_cam_w,
      const std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>>& target_points,
      // const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target,
      const Eigen::Matrix2d& invR) {
    std::vector<boost::shared_ptr<typename CameraT::ReprojectionError>> reprojection_errors;
    AddReprojectionErrorsForViewImplementation(problem, observation, T_cam_w, target_points, invR,
                                               std::ref(reprojection_errors));

    per_view_reprojection_errors_.push_back(reprojection_errors);
  }

  boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera_geometry() const { return camera_geometry_; }

 private:
  // Returning the designvariable and then passing it to another function makes the data types
  // leak outside of the class scope. To avoid that we keep the design variable as a member variable.
  boost::shared_ptr<typename CameraT::Geometry> camera_geometry_;
  typename CameraT::DesignVariable design_variable_;

  // Instead of using the target, we need a way to pass an
  // std::vector<aslam::backend::HomogeneousPoint>
  // Or std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>>
  // And the for loop is over the array instead over a size.
  void AddReprojectionErrorsForViewImplementation(
      boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
      const aslam::cameras::GridCalibrationTargetObservation& observation,
      const aslam::backend::TransformationExpression& T_cam_w,
      // const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target,
      const std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>>& target_points,
      const Eigen::Matrix2d& invR,
      std::optional<std::reference_wrapper<std::vector<boost::shared_ptr<typename CameraT::ReprojectionError>>>>
          reprojection_errors) const {
    for (size_t i = 0; i < target_points.size(); ++i) {
      auto p_target = target_points[i]->toExpression();
      Eigen::Vector2d y;
      bool valid = observation.imagePoint(i, y);
      if (valid) {
        // std::cout << "Target point (homogeneous): " << p_target.toHomogeneous() << std::endl;
        // std::cout << "Point to pass: " << (T_cam_w * p_target).toHomogeneous() << std::endl;
        // Probably I need to store this multiplication to make it work?
        // I did this in the pass.
        auto rerr =
            boost::make_shared<typename CameraT::ReprojectionError>(y, invR, T_cam_w * p_target, design_variable_);
        problem->addErrorTerm(rerr);
        // std::cout << "Predicted measurement when added" << rerr->getPredictedMeasurement()
        //           << std::endl;  // For debugging

        if (reprojection_errors.has_value()) {
          reprojection_errors->get().push_back(rerr);
        }
      }
    }
  }

  // Instead of using the target, we need a way to pass an
  // std::vector<aslam::backend::HomogeneousPoint>
  // Or std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>>
  // And the for loop is over the array instead over a size.
  void AddReprojectionErrorsForViewImplementation(
      boost::shared_ptr<aslam::calibration::OptimizationProblem> problem,
      const aslam::cameras::GridCalibrationTargetObservation& observation,
      const aslam::backend::TransformationExpression& T_cam_w,
      const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target, const Eigen::Matrix2d& invR,
      std::optional<std::reference_wrapper<std::vector<boost::shared_ptr<typename CameraT::ReprojectionError>>>>
          reprojection_errors) const {
    for (size_t i = 0; i < target->size(); ++i) {
      auto p_target = aslam::backend::HomogeneousExpression(sm::kinematics::toHomogeneous(target->point(i)));
      Eigen::Vector2d y;
      bool valid = observation.imagePoint(i, y);
      if (valid) {
        // std::cout << "Target point (homogeneous): " << p_target.toHomogeneous() << std::endl;
        // std::cout << "Point to pass: " << (T_cam_w * p_target).toHomogeneous() << std::endl;
        // Probably I need to store this multiplication to make it work?
        // I did this in the pass.
        auto rerr =
            boost::make_shared<typename CameraT::ReprojectionError>(y, invR, T_cam_w * p_target, design_variable_);
        problem->addErrorTerm(rerr);
        // std::cout << "Predicted measurement when added" << rerr->getPredictedMeasurement()
        //           << std::endl;  // For debugging

        if (reprojection_errors.has_value()) {
          reprojection_errors->get().push_back(rerr);
        }
      }
    }
  }

  std::vector<Eigen::Matrix<double, CameraT::Geometry::KeypointDimension, 1>> to_error_value(
      std::vector<boost::shared_ptr<typename CameraT::ReprojectionError>> reprojection_errors) const {
    std::vector<Eigen::Matrix<double, CameraT::Geometry::KeypointDimension, 1>> errors;
    std::transform(reprojection_errors.begin(), reprojection_errors.end(), std::back_inserter(errors),
                   [](const boost::shared_ptr<typename CameraT::ReprojectionError>& error) {
                     auto corner = error->getMeasurement();
                     //  std::cout << "Corner measurement -- in loop : " << corner.transpose() << std::endl;
                     auto reprojection = error->getPredictedMeasurement();
                     //  std::cout << "Reprojection measurement -- in loop : " << reprojection.transpose() << std::endl;
                     Eigen::Matrix<double, CameraT::Geometry::KeypointDimension, 1> rep_error_value =
                         corner - reprojection;
                     //  std::cout << "Reprojection error value -- in loop : " << rep_error_value.transpose() <<
                     //  std::endl;
                     return rep_error_value;
                   });
    // for (auto& error : errors) {
    //   std::cout << "Reprojection error value: " << error.transpose() << std::endl;
    // }
    return errors;
  }

  template <typename T>
  std::vector<T> flatten(const std::vector<std::vector<T>>& vec_of_vecs) const {
    std::vector<T> flat;

    size_t total_size = 0;
    for (const auto& inner_vec : vec_of_vecs) {
      // std::cout << "Inner vector size: " << inner_vec.size() << std::endl;
      total_size += inner_vec.size();
    }
    flat.reserve(total_size);
    // std::cout << "Total flattened vector size: " << total_size << std::endl;

    // Insert the contents of each inner vector into the flat vector
    for (const auto& inner_vec : vec_of_vecs) {
      flat.insert(flat.end(), inner_vec.begin(), inner_vec.end());
    }

    return flat;
  }

  // Maybe get the dimension? some how from the client side...
  // Or how would we now the size of the returned vector?
  // Maybe we need a dynamic Matrix Eigen::MatrixXd...
  //   std::vector<Eigen::Matrix<double, CameraT::Geometry::KeypointDimension, 1>> GetReprojectionErrorsMean()
  // I'd like to have a getter more than a print
  void PrintReprojectionErrorsMean() const {
    // TODO(frneer): If the per_view_reprojection_errors_ is empty, return sth or raise?
    if (per_view_reprojection_errors_.empty()) {
      std::cout << "No reprojection errors stored." << std::endl;
      return;
    }
    std::cout << "Calculating mean reprojection error over " << per_view_reprojection_errors_.size() << " views."
              << std::endl;

    // Flatten the per_view_reprojection_errors_ into a single vector
    // [[corner1, corner2], [corner3]] -> [corner1, corner2, corner3]
    auto all_errors = flatten(per_view_reprojection_errors_);
    if (all_errors.empty()) {
      std::cout << "No reprojection errors to compute mean." << std::endl;
      return;
    }

    // Get the error values
    auto error_values = to_error_value(all_errors);

    // Compute the mean reprojection error
    // nit: maybe std::accumulate?
    Eigen::Matrix<double, CameraT::Geometry::KeypointDimension, 1> mean_error =
        Eigen::Matrix<double, CameraT::Geometry::KeypointDimension, 1>::Zero();
    for (const auto& error : error_values) {
      mean_error += error;
    }
    mean_error /= static_cast<double>(error_values.size());
    for (auto v : mean_error.reshaped()) {
      std::cout << "Reprojection error: " << v << std::endl;
    }
  }

  std::vector<Eigen::Matrix<double, CameraT::Geometry::KeypointDimension, 1>> GetErrorValues() const {
    // TODO(frneer): If the per_view_reprojection_errors_ is empty, return sth or raise?
    if (per_view_reprojection_errors_.empty()) {
      std::cout << "No reprojection errors stored." << std::endl;
      return {};
    }
    std::cout << "Calculating mean reprojection error over " << per_view_reprojection_errors_.size() << " views."
              << std::endl;

    // Flatten the per_view_reprojection_errors_ into a single vector
    // [[corner1, corner2], [corner3]] -> [corner1, corner2, corner3]
    auto all_errors = flatten(per_view_reprojection_errors_);
    if (all_errors.empty()) {
      std::cout << "No reprojection errors to compute mean." << std::endl;
      return {};
    }

    // Get the error values
    auto error_values = to_error_value(all_errors);
    return error_values;
  }

  void PrintReprojectionErrorStatistics() const {
    // This would come from your GetFinalReprojectionErrors() method
    auto error_values = GetErrorValues();

    if (error_values.empty()) {
      std::cout << "No reprojection errors to compute statistics." << std::endl;
      return;
    }

    // Use a type alias for clarity
    using ErrorVectorT = Eigen::Matrix<double, CameraT::Geometry::KeypointDimension, 1>;

    // --- 1. Compute the Mean (First Pass) ---
    ErrorVectorT sum_of_errors =
        std::accumulate(error_values.begin(), error_values.end(),
                        ErrorVectorT::Zero().eval()  // .eval() helps the compiler with type deduction
        );
    ErrorVectorT mean_error = sum_of_errors / static_cast<double>(error_values.size());

    // --- 2. Compute the Standard Deviation (Second Pass) ---
    // We need at least 2 samples to compute a meaningful sample standard deviation.
    ErrorVectorT std_dev = ErrorVectorT::Zero();
    if (error_values.size() > 1) {
      ErrorVectorT sum_of_squared_diffs = ErrorVectorT::Zero();
      for (const auto& error : error_values) {
        ErrorVectorT diff = error - mean_error;
        // Use .array() for element-wise operations, then convert back to a matrix
        sum_of_squared_diffs += diff.array().square().matrix();
      }

      // Sample Variance (divide by N-1)
      ErrorVectorT variance = sum_of_squared_diffs / (static_cast<double>(error_values.size()) - 1);

      // Standard Deviation is the square root of variance
      std_dev = variance.array().sqrt().matrix();
    }

    // --- 3. Print the Results ---
    std::cout << "Reprojection Error Statistics (" << error_values.size() << " points):" << std::endl;
    // The .transpose() makes it print as a row vector [x, y]
    std::cout << "  Mean (px):      " << mean_error.transpose() << std::endl;
    std::cout << "  Std Dev (px):   " << std_dev.transpose() << std::endl;
    std::cout << "  RMSE (px):      " << sum_of_errors.norm() / std::sqrt(error_values.size()) << std::endl;
  }

  // Store reprojection errors for later use (e.g., statistics computation)
  std::vector<std::vector<boost::shared_ptr<typename CameraT::ReprojectionError>>> per_view_reprojection_errors_;
};

}  // namespace kalibr2
