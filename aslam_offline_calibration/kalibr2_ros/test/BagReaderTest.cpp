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
  const size_t message_count = reader.MessageCount();
  size_t images_processed = 0;

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

struct CameraConfig {
  std::unique_ptr<kalibr2::ImageReader> reader;
  std::string model;
  std::optional<double> focal_length;  // Optional focal length for calibration
};

struct CalibrationConfig {
  std::vector<CameraConfig> cameras;
  boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target;
};

TEST_F(BagReaderTestFixture, DISABLED_MultiThreadReading) {
  CalibrationConfig config;
  config.target = std::make_unique<aslam::cameras::GridCalibrationTargetAprilgrid>(5, 6, 0.088, 0.2954);
  config.cameras.emplace_back(CameraConfig{kalibr2::ros::BagImageReaderFactory::create(bag_path, "/BFS_25037058/image"),
                                           "DistortedPinhole", 881.0});
  config.cameras.emplace_back(CameraConfig{kalibr2::ros::BagImageReaderFactory::create(bag_path, "/BFS_25037070/image"),
                                           "DistortedPinhole", 881.0});

  std::vector<boost::shared_ptr<kalibr2::CameraCalibratorBase>> camera_calibrators;
  for (const auto& camera_config : config.cameras) {
    auto calibrator = boost::make_shared<kalibr2::CameraCalibrator<kalibr2::models::DistortedPinhole>>();
    camera_calibrators.push_back(calibrator);
  }

  // |---- Extract observations for each camera ----|
  std::vector<std::vector<aslam::cameras::GridCalibrationTargetObservation>> observations_by_camera;
  observations_by_camera.resize(camera_calibrators.size());

  // Helper lambda that calls the existing get_observations_from_camera and writes the result to 'out'.
  auto get_observations_thread_fn = [](kalibr2::ImageReader& reader, const aslam::cameras::GridDetector& detector,
                                       size_t min_num_observations,
                                       std::vector<aslam::cameras::GridCalibrationTargetObservation>& out) {
    out = get_observations_from_camera(reader, detector, min_num_observations);
  };

  std::vector<std::thread> threads;
  for (size_t camera_id = 0; camera_id < camera_calibrators.size(); ++camera_id) {
    // Prepare per-thread arguments
    const auto& camera_config = config.cameras.at(camera_id);
    auto detector = aslam::cameras::GridDetector(camera_calibrators.at(camera_id)->camera_geometry(), config.target);
    auto max_observations = 10;

    // Launch thread that runs the helper which delegates to get_observations_from_camera and stores into the provided
    // reference.
    threads.emplace_back(get_observations_thread_fn, std::ref(*camera_config.reader), detector, max_observations,
                         std::ref(observations_by_camera.at(camera_id)));
  }

  for (auto& thread : threads) {
    thread.join();
  }

  // Optional: sanity check that each camera produced some observations (adjust as needed)
  for (size_t i = 0; i < observations_by_camera.size(); ++i) {
    std::cout << "Camera " << i << " collected " << observations_by_camera[i].size() << " observations." << std::endl;
  }
}

TEST_F(BagReaderTestFixture, IntegrationMultipleCameras) {
  // | --- User side setup --- |
  CalibrationConfig config;
  auto target_options = aslam::cameras::GridCalibrationTargetAprilgridv2::AprilgridOptionsv2();
  target_options.detectorParameters->markerBorderBits = 2;
  config.target =
      std::make_unique<aslam::cameras::GridCalibrationTargetAprilgridv2>(5, 6, 0.088, 0.2954, target_options);
  config.cameras.emplace_back(CameraConfig{kalibr2::ros::BagImageReaderFactory::create(bag_path, "/BFS_25037058/image"),
                                           "DistortedPinhole", 881.0});
  config.cameras.emplace_back(CameraConfig{kalibr2::ros::BagImageReaderFactory::create(bag_path, "/BFS_25037070/image"),
                                           "DistortedPinhole", 881.0});

  std::vector<boost::shared_ptr<kalibr2::CameraCalibratorBase>> camera_calibrators;
  for (const auto& camera_config : config.cameras) {
    auto calibrator = boost::make_shared<kalibr2::CameraCalibrator<kalibr2::models::DistortedPinhole>>();
    camera_calibrators.push_back(calibrator);
  }

  // |---- Extract observations for each camera ----|
  std::vector<std::vector<aslam::cameras::GridCalibrationTargetObservation>> observations_by_camera;
  observations_by_camera.resize(camera_calibrators.size());

  // Use multithreading to fill observations_by_camera in parallel (same pattern as MultiThreadReading).
  // const size_t max_observations = 20000;
  const size_t max_observations = 10;
  std::vector<std::thread> threads;
  threads.reserve(camera_calibrators.size());

  for (size_t camera_id = 0; camera_id < camera_calibrators.size(); ++camera_id) {
    // Capture camera_id by value to avoid iterator-capture issues.
    const size_t id = camera_id;
    threads.emplace_back([&config, &camera_calibrators, &observations_by_camera, id, max_observations]() {
      const auto& camera_config = config.cameras.at(id);
      auto detector = aslam::cameras::GridDetector(camera_calibrators.at(id)->camera_geometry(), config.target);
      observations_by_camera.at(id) = get_observations_from_camera(*camera_config.reader, detector, max_observations);
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  for (size_t i = 0; i < observations_by_camera.size(); ++i) {
    std::cout << "Camera " << i << " collected " << observations_by_camera[i].size() << " observations." << std::endl;
  }

  // |---- Get initial intrinsic guess for each camera ----|
  for (size_t camera_id = 0; camera_id < camera_calibrators.size(); ++camera_id) {
    const auto& camera_config = config.cameras.at(camera_id);
    auto detector = aslam::cameras::GridDetector(camera_calibrators.at(camera_id)->camera_geometry(), config.target);
    bool success =
        kalibr2::tools::CalibrateSingleCamera(observations_by_camera.at(camera_id), camera_calibrators.at(camera_id),
                                              config.target, camera_config.focal_length);
    ASSERT_TRUE(success) << "Failed to calibrate intrinsics from observations for camera ID: " << camera_id;
  }

  // | ---- Sync observations across cameras ----|
  aslam::Duration tolerance(0.01);
  auto synced_sets = std::vector<kalibr2::SyncedSet>();
  for (const auto& sync_set : kalibr2::SynchronizedObservationView(observations_by_camera, tolerance)) {
    synced_sets.push_back(sync_set);
    std::cout << "\n--- Sync Set ---" << std::endl;
    for (size_t i = 0; i < sync_set.size(); ++i) {
      if (sync_set[i].has_value()) {
        std::cout << "  Source " << i + 1 << ": " << sync_set[i].value().time() << std::endl;
      } else {
        std::cout << "  Source " << i + 1 << ": No observation" << std::endl;
      }
    }
  }

  // | ---- Build Graph ----|
  auto graph = kalibr2::BuildCameraGraph(synced_sets);

  // |||| THIS IS WHERE THE original call calls getInitialGuesses to get the baselines ||||
  // | ---- Perform Dijkstra's Algorithm ----|
  constexpr size_t start_node_idx = 0;
  common_robotics_utilities::simple_graph_search::DijkstrasResult result =
      common_robotics_utilities::simple_graph_search::PerformDijkstrasAlgorithm(graph, start_node_idx);

  // | ---- Stereo Calibration Best Pairs ----|
  std::map<std::pair<size_t, size_t>, sm::kinematics::Transformation> optimal_baselines;
  for (size_t i = 0; i < camera_calibrators.size(); ++i) {
    if (i == start_node_idx) {
      continue;  // Skip the start node
    } else {
      auto best_camera_pair = result.GetPreviousIndex(i);
      std::cout << "Best pair for camera " << i << ": " << best_camera_pair << std::endl;
      std::cout << "Distance to camera " << i << ": " << result.GetNodeDistance(i) << std::endl;
      // Stereo calibrate pair
      auto tf = kalibr2::tools::CalibrateStereoPair(
          camera_calibrators.at(i), camera_calibrators.at(best_camera_pair),
          kalibr2::GetAllObservationsFromSource(synced_sets, i),
          kalibr2::GetAllObservationsFromSource(synced_sets, best_camera_pair), config.target);
      optimal_baselines[{i, best_camera_pair}] = tf;
    }
  }

  for (size_t i = 0; i < camera_calibrators.size() - 1; ++i) {
    std::cout << "Checking for transform between camera " << i << " and " << i + 1 << std::endl;
    // If the transform is already in the map, continue
    auto tf_it = optimal_baselines.find({i, i + 1});
    if (tf_it != optimal_baselines.end()) {
      std::cout << "Transform already exists between camera " << i << " and " << i + 1 << std::endl;
      continue;
    }

    // If the inverse transform is in the map, add the inverse and continue
    tf_it = optimal_baselines.find({i + 1, i});
    if (tf_it != optimal_baselines.end()) {
      std::cout << "Inverse transform found between camera " << i + 1 << " and " << i << std::endl;
      optimal_baselines[{i, i + 1}] = tf_it->second.inverse();
      continue;
    }

    // Otherwise, compute the transform using Dijkstra's result
    auto tf = kalibr2::GetTransform(optimal_baselines, result, i, i + 1);
    optimal_baselines[{i, i + 1}] = tf;
  }

  std::vector<sm::kinematics::Transformation> baseline_guesses;
  for (size_t i = 0; i < camera_calibrators.size() - 1; ++i) {
    auto tf_it = optimal_baselines.find({i, i + 1});
    ASSERT_TRUE(tf_it != optimal_baselines.end()) << "No transform found for camera pair: " << i << " and " << i + 1;
    baseline_guesses.push_back(tf_it->second);
  }

  // | ---- Refine guess in full batch optimization ----|
  auto baselines =
      kalibr2::tools::CalibrateMultiCameraRig(camera_calibrators, synced_sets, config.target, baseline_guesses);

  // |||| (END) THIS IS WHERE THE original call calls getInitialGuesses to get the baselines ||||

  // | ----
  constexpr int CALIBRATION_GROUP_ID = 0;
  auto ic_options = aslam::calibration::IncrementalEstimator::Options();
  ic_options.infoGainDelta = 0.2;  // TODO(frneer): This is coming from the cli
  ic_options.checkValidity = true;
  ic_options.verbose = true;  // same here

  auto linear_solver_options = aslam::calibration::LinearSolverOptions();
  linear_solver_options.columnScaling = true;
  linear_solver_options.verbose = true;
  linear_solver_options.epsSVD = 1e-6;

  auto optimizer_options = aslam::backend::Optimizer2Options();
  // original code maxes the number based on available cores
  optimizer_options.maxIterations = 50;
  optimizer_options.nThreads = 1;
  optimizer_options.verbose = true;
  auto estimator = aslam::calibration::IncrementalEstimator(CALIBRATION_GROUP_ID, ic_options, linear_solver_options,
                                                            optimizer_options);

  // TODO(frneer): randomize the order of sync sets on request
  // std::random_shuffle(synced_sets.begin(), synced_sets.end());
  // std::vector<boost::shared_ptr<aslam::calibration::OptimizationProblem>> batch_problems;
  // TODO(frneer): Do we need to store every item of the structs?
  std::vector<kalibr2::tools::BatchProblemStruct> batch_problems;
  for (auto& synced_set : synced_sets) {
    auto T_tc_guess = kalibr2::tools::getTargetPoseGuess(camera_calibrators, synced_set, baseline_guesses);
    // auto batch_problem = OptimizationProblem();
    std::cout << "\n--- New Batch Problem ---" << std::endl;
    auto batch_problem_struct =
        kalibr2::tools::CreateBatchProblem(camera_calibrators, synced_set, T_tc_guess, baseline_guesses, config.target);
    std::cout << "Created batch problem with " << std::endl;
    // define the problem as in original code
    auto batch_return_value = estimator.addBatch(batch_problem_struct.problem);
    std::cout << "Attempt to add batch" << std::endl;
    if (batch_return_value.numIterations > optimizer_options.maxIterations) {
      throw std::runtime_error("Optimizer reached max iterations. Something went wrong.");
    }

    batch_problems.push_back(batch_problem_struct);
    if (batch_return_value.batchAccepted) {
      std::cout << "Batch accepted. Information gain: " << batch_return_value.informationGain << std::endl;
      // batch_problems.push_back(batch_problem_struct.problem);
    } else {
      std::cout << "Batch rejected." << std::endl;
    }
  }

  std::cout << "\n--- Final Camera Parameters ---" << std::endl;
  for (auto& camera_calibrator : camera_calibrators) {
    Eigen::MatrixXd k;
    camera_calibrator->camera_geometry()->getParameters(k, true, true, false);
    std::cout << "Intrinsics: " << std::endl;
    std::cout << k << std::endl;
    camera_calibrator->PrintReprojectionErrorStatistics();
    // std::cout << "Reprojection error: " <<  << std::endl;
  }
  std::cout << "Extrinsics: " << std::endl;
  for (const auto& baseline : baseline_guesses) {
    std::cout << "Translation: " << std::endl;
    std::cout << baseline.t() << std::endl;
    std::cout << "Rotation: " << std::endl;
    std::cout << baseline.q() << std::endl;
  }
}

}  // namespace
