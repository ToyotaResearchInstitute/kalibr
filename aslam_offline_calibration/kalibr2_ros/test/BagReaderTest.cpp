#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <gtest/gtest.h>
#include <kalibr2/CalibrationTools.hpp>
#include <kalibr2/CameraGraph.hpp>
#include <kalibr2/CameraModels.hpp>
#include <kalibr2/Image.hpp>
#include <kalibr2/SynchronizedObservationView.hpp>
#include <kalibr2_ros/BagReader.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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

TEST_F(BagReaderTestFixture, Integration) {
  auto reader = kalibr2::ros::BagImageReaderFactory::create(bag_path, topic);

  constexpr int rows = 5;
  constexpr int cols = 6;
  constexpr double tag_size = 0.088;
  constexpr double tag_spacing = 0.2954;
  auto target_grid =
      boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(rows, cols, tag_size, tag_spacing);
  auto geometry = boost::make_shared<kalibr2::models::DistortedPinhole::Geometry>();
  auto detector = aslam::cameras::GridDetector(geometry, target_grid);

  constexpr size_t min_num_observation = 10;
  std::vector<aslam::cameras::GridCalibrationTargetObservation> observations;

  while (reader->HasNext()) {
    kalibr2::Image img = reader->ReadNext();
    ASSERT_FALSE(img.image.empty());

    auto observation = kalibr2::ToObservation(img, detector);

    if (observation.has_value()) {
      observations.push_back(observation.value());
      std::cout << "Found target in image with timestamp: " << img.timestamp.toNSec() << std::endl;
      if (observations.size() >= min_num_observation) {
        std::cout << "Enough observations collected: " << observations.size() << std::endl;
        break;  // Stop after collecting enough observations
      }
    } else {
      std::cout << "No target found in image with timestamp: " << img.timestamp.toNSec() << std::endl;
    }
  }

  constexpr double focal_length = 881.0;
  bool success = kalibr2::tools::CalibrateSingleCamera<kalibr2::models::DistortedPinhole>(observations, geometry,
                                                                                          target_grid, focal_length);

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

TEST_F(BagReaderTestFixture, IntegrationMultipleCameras) {
  // | --- User side setup --- |
  CalibrationConfig config;
  config.target = std::make_unique<aslam::cameras::GridCalibrationTargetAprilgrid>(5, 6, 0.088, 0.2954);
  config.cameras.emplace_back(CameraConfig{kalibr2::ros::BagImageReaderFactory::create(bag_path, "/BFS_25037070/image"),
                                           "DistortedPinhole", 881.0});

  config.cameras.emplace_back(CameraConfig{kalibr2::ros::BagImageReaderFactory::create(bag_path, "/BFS_25037058/image"),
                                           "DistortedPinhole", 881.0});

  std::vector<boost::shared_ptr<aslam::cameras::CameraGeometryBase>> camera_geometries;
  for (const auto& camera_config : config.cameras) {
    auto geometry = boost::make_shared<kalibr2::models::DistortedPinhole::Geometry>();
    camera_geometries.push_back(geometry);
  }

  // |---- Extract observations for each camera ----|
  std::vector<std::vector<aslam::cameras::GridCalibrationTargetObservation>> observations_by_camera;
  observations_by_camera.resize(config.cameras.size());

  for (size_t camera_id = 0; camera_id < config.cameras.size(); ++camera_id) {
    const auto& camera_config = config.cameras.at(camera_id);
    auto detector = aslam::cameras::GridDetector(camera_geometries.at(camera_id), config.target);
    auto max_observations = 10;
    auto observations = 0;

    while (camera_config.reader->HasNext()) {
      kalibr2::Image img = camera_config.reader->ReadNext();
      ASSERT_FALSE(img.image.empty());

      auto observation = kalibr2::ToObservation(img, detector);

      if (observation.has_value()) {
        observations_by_camera.at(camera_id).push_back(observation.value());
        std::cout << "Found target in image with timestamp: " << img.timestamp.toNSec() << std::endl;
        observations++;
        if (observations == max_observations) {
          std::cout << "Enough observations collected for camera ID: " << camera_id << std::endl;
          break;  // Stop after collecting enough observations for this camera
        }
      } else {
        std::cout << "No target found in image with timestamp: " << img.timestamp.toNSec() << std::endl;
      }
    }
  }

  // |---- Get initial intrinsic guess for each camera ----|
  for (size_t camera_id = 0; camera_id < config.cameras.size(); ++camera_id) {
    const auto& camera_config = config.cameras.at(camera_id);
    auto detector = aslam::cameras::GridDetector(camera_geometries.at(camera_id), config.target);
    bool success = kalibr2::tools::CalibrateSingleCamera<kalibr2::models::DistortedPinhole>(
        observations_by_camera.at(camera_id), camera_geometries.at(camera_id), config.target,
        camera_config.focal_length);
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

  // | ---- Perform Dijkstra's Algorithm ----|
  constexpr size_t start_node_idx = 0;
  common_robotics_utilities::simple_graph_search::DijkstrasResult result =
      common_robotics_utilities::simple_graph_search::PerformDijkstrasAlgorithm(graph, start_node_idx);

  // | ---- Stereo Calibration Best Pairs ----|
  for (size_t i = 0; i < config.cameras.size(); ++i) {
    if (i == start_node_idx) {
      continue;  // Skip the start node
    } else {
      auto best_camera_pair = result.GetPreviousIndex(i);
      std::cout << "Best pair for camera " << i << ": " << best_camera_pair << std::endl;
      std::cout << "Distance to camera " << i << ": " << result.GetNodeDistance(i) << std::endl;
      // Stereo calibrate pair
      auto tf =
          kalibr2::tools::CalibrateStereoPair<kalibr2::models::DistortedPinhole, kalibr2::models::DistortedPinhole>(
              camera_geometries.at(i), camera_geometries.at(best_camera_pair),
              kalibr2::GetAllObservationsFromSource(synced_sets, i),
              kalibr2::GetAllObservationsFromSource(synced_sets, best_camera_pair), config.target);
    }
  }
}

}  // namespace
