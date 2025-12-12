#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgridv2.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <gtest/gtest.h>
#include <kalibr2/CalibrationTools.hpp>
#include <kalibr2/CameraCalibrator.hpp>
#include <kalibr2/CameraGraph.hpp>
#include <kalibr2/CameraModels.hpp>
#include <kalibr2/Image.hpp>
#include <kalibr2/SynchronizedObservationView.hpp>
#include <kalibr2_ros/BagReader.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Temporal includes
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/calibration/core/IncrementalEstimator.h>

namespace {

TEST(BagReaderTest, InvalidBagFile) {
  std::string invalid_bag_path = TEST_DATA_DIR "/invalid_bag.mcap";
  EXPECT_THROW(kalibr2::ros::BagImageReaderFactory::create(invalid_bag_path, "/BFS_25037070/image"),
               std::runtime_error);
}

class BagReaderTestFixture : public ::testing::Test {
 protected:
  const std::string bag_path = TEST_DATA_DIR "/rosbag2_2025_06_11-12_00_21_0.mcap";
  const std::string topic = "/BFS_25037070/image";
  void SetUp() override {
    if (!std::filesystem::exists(bag_path)) {
      GTEST_SKIP() << "Bag file does not exist: " << bag_path;
    }
  }
};

TEST_F(BagReaderTestFixture, InvalidTopic) {
  EXPECT_THROW(kalibr2::ros::BagImageReaderFactory::create(bag_path, "/invalid_topic"), std::runtime_error);
}

TEST_F(BagReaderTestFixture, CanInitializeBagReader) {
  auto reader = kalibr2::ros::BagImageReaderFactory::create(bag_path, topic);
}

TEST_F(BagReaderTestFixture, CanReadSingleImageSingleTopic) {
  auto reader = kalibr2::ros::BagImageReaderFactory::create(bag_path, topic);
  kalibr2::Image img = reader->ReadNext();
  std::cout << "Message count: " << reader->MessageCount() << std::endl;
  ASSERT_FALSE(img.image.empty());
}

TEST_F(BagReaderTestFixture, DISABLED_CanReadMultipleImagesSingleTopic) {
  auto reader = kalibr2::ros::BagImageReaderFactory::create(bag_path, topic);
  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

  while (reader->HasNext()) {
    kalibr2::Image img = reader->ReadNext();
    ASSERT_FALSE(img.image.empty());
    cv::imshow("Image", img.image);
    cv::waitKey(1);
  }
}

TEST_F(BagReaderTestFixture, DISABLED_CanReadMultipleImagesMultipleTopics) {
  const std::string other_topic = "/BFS_24293899/image";
  std::array<std::string, 2> topics = {topic, other_topic};
  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

  for (const auto& topic : topics) {
    auto reader = kalibr2::ros::BagImageReaderFactory::create(bag_path, topic);
    while (reader->HasNext()) {
      kalibr2::Image img = reader->ReadNext();
      ASSERT_FALSE(img.image.empty());
      cv::imshow("Image", img.image);
      cv::waitKey(1);
    }
  }
}

TEST_F(BagReaderTestFixture, DISABLED_CanDetectMultipleImagesSingleTopic) {
  auto reader = kalibr2::ros::BagImageReaderFactory::create(bag_path, topic);
  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

  constexpr int rows = 5;
  constexpr int cols = 6;
  constexpr double tag_size = 0.088;
  constexpr double tag_spacing = 0.2954;
  auto target_grid =
      boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(rows, cols, tag_size, tag_spacing);
  auto geometry = boost::make_shared<kalibr2::models::DistortedPinhole::Geometry>();
  auto detector = aslam::cameras::GridDetector(geometry, target_grid);

  while (reader->HasNext()) {
    kalibr2::Image img = reader->ReadNext();
    ASSERT_FALSE(img.image.empty());
    cv::imshow("Image", img.image);
    cv::waitKey(1);

    auto observation = kalibr2::ToObservation(img, detector);

    if (observation.has_value()) {
      std::cout << "Found target in image with timestamp: " << img.timestamp.toNSec() << std::endl;
    } else {
      std::cout << "No target found in image with timestamp: " << img.timestamp.toNSec() << std::endl;
    }
  }
}

std::vector<aslam::cameras::GridCalibrationTargetObservation> get_observations_from_camera(
    kalibr2::ImageReader& reader, const aslam::cameras::GridDetector& detector, size_t min_num_observations) {
  std::vector<aslam::cameras::GridCalibrationTargetObservation> observations;

  while (reader.HasNext()) {
    kalibr2::Image img = reader.ReadNext();
    auto observation = kalibr2::ToObservation(img, detector);

    if (observation.has_value()) {
      observations.push_back(observation.value());
      if (observations.size() >= min_num_observations) {
        std::cout << "Enough observations collected" << std::endl;
        break;  // Stop after collecting enough observations
      }
    }
  }

  return observations;
}

TEST_F(BagReaderTestFixture, DISABLED_ProfileCameraReading) {
  auto reader = kalibr2::ros::BagImageReaderFactory::create(bag_path, topic);

  constexpr int rows = 5;
  constexpr int cols = 6;
  constexpr double tag_size = 0.088;
  constexpr double tag_spacing = 0.2954;
  auto target_grid =
      boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(rows, cols, tag_size, tag_spacing);
  boost::shared_ptr<kalibr2::CameraCalibratorBase> calibrator =
      boost::make_shared<kalibr2::CameraCalibrator<kalibr2::models::DistortedPinhole>>();
  auto detector = aslam::cameras::GridDetector(calibrator->camera_geometry(), target_grid);

  constexpr size_t min_num_observation = 10;

  auto observations = get_observations_from_camera(*reader, detector, min_num_observation);
}

TEST_F(BagReaderTestFixture, DISABLED_Integration) {
  auto reader = kalibr2::ros::BagImageReaderFactory::create(bag_path, topic);

  constexpr int rows = 5;
  constexpr int cols = 6;
  constexpr double tag_size = 0.088;
  constexpr double tag_spacing = 0.2954;
  auto target_grid =
      boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(rows, cols, tag_size, tag_spacing);
  boost::shared_ptr<kalibr2::CameraCalibratorBase> calibrator =
      boost::make_shared<kalibr2::CameraCalibrator<kalibr2::models::DistortedPinhole>>();
  auto detector = aslam::cameras::GridDetector(calibrator->camera_geometry(), target_grid);

  constexpr size_t min_num_observation = 10;

  auto observations = get_observations_from_camera(*reader, detector, min_num_observation);

  constexpr double focal_length = 881.0;
  bool success = kalibr2::tools::CalibrateSingleCamera(observations, calibrator, target_grid, focal_length);
  calibrator->PrintReprojectionErrorsMean();

  ASSERT_TRUE(success) << "Failed to calibrate intrinsics from observations";
}

TEST_F(BagReaderTestFixture, SingleBatchOptimizationStandalone) {
  // This test creates a single batch problem and optimizes it directly
  // (without IncrementalEstimator) to diagnose batch construction issues

  // Setup target
  constexpr int rows = 5;
  constexpr int cols = 6;
  constexpr double tag_size = 0.088;
  constexpr double tag_spacing = 0.2954;
  auto target_grid =
      boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(rows, cols, tag_size, tag_spacing);

  // Setup two cameras
  const std::string topic1 = "/BFS_25037070/image";
  const std::string topic2 = "/BFS_25037058/image";

  boost::shared_ptr<kalibr2::CameraCalibratorBase> calibrator1 =
      boost::make_shared<kalibr2::CameraCalibrator<kalibr2::models::DistortedPinhole>>();
  boost::shared_ptr<kalibr2::CameraCalibratorBase> calibrator2 =
      boost::make_shared<kalibr2::CameraCalibrator<kalibr2::models::DistortedPinhole>>();

  std::vector<boost::shared_ptr<kalibr2::CameraCalibratorBase>> camera_calibrators = {calibrator1, calibrator2};

  // Get observations from both cameras
  auto reader1 = kalibr2::ros::BagImageReaderFactory::create(bag_path, topic1);
  auto reader2 = kalibr2::ros::BagImageReaderFactory::create(bag_path, topic2);

  auto detector1 = aslam::cameras::GridDetector(calibrator1->camera_geometry(), target_grid);
  auto detector2 = aslam::cameras::GridDetector(calibrator2->camera_geometry(), target_grid);

  constexpr size_t min_num_observation = 50;
  auto observations1 = get_observations_from_camera(*reader1, detector1, min_num_observation);
  auto observations2 = get_observations_from_camera(*reader2, detector2, min_num_observation);

  std::cout << "Camera 1 observations: " << observations1.size() << std::endl;
  std::cout << "Camera 2 observations: " << observations2.size() << std::endl;

  // Calibrate intrinsics for both cameras
  constexpr double focal_length = 881.0;
  bool success1 = kalibr2::tools::CalibrateSingleCamera(observations1, calibrator1, target_grid, focal_length);
  bool success2 = kalibr2::tools::CalibrateSingleCamera(observations2, calibrator2, target_grid, focal_length);

  ASSERT_TRUE(success1) << "Failed to calibrate camera 1 intrinsics";
  ASSERT_TRUE(success2) << "Failed to calibrate camera 2 intrinsics";

  std::cout << "Camera 1 intrinsics calibrated" << std::endl;
  calibrator1->PrintReprojectionErrorsMean();
  std::cout << "Camera 2 intrinsics calibrated" << std::endl;
  calibrator2->PrintReprojectionErrorsMean();

  // Synchronize observations
  std::vector<std::vector<aslam::cameras::GridCalibrationTargetObservation>> observations_by_camera = {observations1,
                                                                                                       observations2};
  auto synced_sets = std::vector<kalibr2::SyncedSet>();
  for (const auto& sync_set : kalibr2::SynchronizedObservationView(observations_by_camera, aslam::Duration(0.02))) {
    synced_sets.push_back(sync_set);
  }

  std::cout << "Synced sets: " << synced_sets.size() << std::endl;
  ASSERT_GT(synced_sets.size(), 0) << "No synchronized observations found";

  // Get baseline guess from stereo calibration
  // Convert observations to std::optional format expected by CalibrateStereoPair
  auto observations1_opt = kalibr2::GetAllObservationsFromSource(synced_sets, 0);
  auto observations2_opt = kalibr2::GetAllObservationsFromSource(synced_sets, 1);
  auto baseline_guess =
      kalibr2::tools::CalibrateStereoPair(calibrator1, calibrator2, observations1_opt, observations2_opt, target_grid);

  std::cout << "Baseline guess computed" << std::endl;
  std::cout << "Translation: " << baseline_guess.t().transpose() << std::endl;

  // Create baseline design variables
  auto q_Dv = boost::make_shared<aslam::backend::RotationQuaternion>(baseline_guess.q());
  q_Dv->setActive(true);

  auto t_Dv = boost::make_shared<aslam::backend::EuclideanPoint>(baseline_guess.t());
  t_Dv->setActive(true);

  auto baseline_transformation =
      boost::make_shared<aslam::backend::TransformationBasic>(q_Dv->toExpression(), t_Dv->toExpression());

  std::vector<kalibr2::tools::PoseDesignVariables> baseline_pose_dvs;
  baseline_pose_dvs.push_back({q_Dv, t_Dv, baseline_transformation});

  // Create landmark design variables
  std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>> landmark_dvs;
  for (size_t i = 0; i < target_grid->size(); ++i) {
    landmark_dvs.push_back(
        boost::make_shared<aslam::backend::HomogeneousPoint>(sm::kinematics::toHomogeneous(target_grid->point(i))));
  }

  // Pick the first synced observation to create a batch
  auto& first_synced_set = synced_sets[0];

  std::cout << "\n--- Creating Batch Problem ---" << std::endl;
  std::cout << "Synced set has " << first_synced_set.size() << " cameras" << std::endl;
  std::cout << "Camera 0 has observation: " << first_synced_set[0].has_value() << std::endl;
  std::cout << "Camera 1 has observation: " << first_synced_set[1].has_value() << std::endl;

  // Get target pose guess
  std::vector<sm::kinematics::Transformation> baselines;
  baselines.push_back(baseline_guess);
  auto T_tc_guess = kalibr2::tools::getTargetPoseGuess(camera_calibrators, first_synced_set, baselines);

  std::cout << "Target pose guess computed" << std::endl;

  // Create the batch problem using the shared design variables
  auto batch_problem_struct = kalibr2::tools::CreateBatchProblem(camera_calibrators, first_synced_set, T_tc_guess,
                                                                 baseline_pose_dvs, landmark_dvs);

  std::cout << "\n--- Batch Problem Created ---" << std::endl;
  std::cout << "Number of design variables: " << batch_problem_struct.problem->numDesignVariables() << std::endl;
  std::cout << "Number of error terms: " << batch_problem_struct.problem->numErrorTerms() << std::endl;

  // Setup optimizer options
  auto optimizer_options = aslam::backend::Optimizer2Options();
  optimizer_options.maxIterations = 50;
  optimizer_options.nThreads = 1;
  optimizer_options.verbose = true;
  // Create a BlockCholeskyLinearSystemSolver with "cholmod" as the solver type
  optimizer_options.linearSystemSolver = boost::make_shared<aslam::backend::BlockCholeskyLinearSystemSolver>("cholmod");

  std::cout << "\n--- Running Optimization ---" << std::endl;
  std::cout << "Max iterations: " << optimizer_options.maxIterations << std::endl;

  // Create optimizer and optimize
  aslam::backend::Optimizer2 optimizer(optimizer_options);
  optimizer.setProblem(batch_problem_struct.problem);

  try {
    auto result = optimizer.optimize();
    std::cout << "\n--- Optimization Result ---" << std::endl;
    if (result.linearSolverFailure) {
      std::cout << "FAILED: Linear solver failure" << std::endl;
    } else {
      std::cout << "SUCCESS: Optimization completed" << std::endl;
    }
    std::cout << "Iterations: " << result.iterations << " (failed: " << result.failedIterations << ")" << std::endl;
    std::cout << "Cost: " << result.JStart << " -> " << result.JFinal << std::endl;
    std::cout << "Final dx norm: " << result.dXFinal << std::endl;
    std::cout << "Final dJ: " << result.dJFinal << std::endl;

    // Print optimized baseline
    sm::kinematics::Transformation optimized_baseline(baseline_transformation->toExpression().toTransformationMatrix());
    std::cout << "Optimized baseline translation: " << optimized_baseline.t().transpose() << std::endl;
    std::cout << "Initial baseline translation: " << baseline_guess.t().transpose() << std::endl;

  } catch (const std::exception& e) {
    std::cout << "\n--- Optimization FAILED ---" << std::endl;
    std::cout << "Exception: " << e.what() << std::endl;
    FAIL() << "Optimization threw exception: " << e.what();
  }
}

}  // namespace
