#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <gtest/gtest.h>
#include <kalibr2/CalibrationTools.hpp>
#include <kalibr2/CameraModels.hpp>
#include <kalibr2/Image.hpp>
#include <kalibr2_ros/BagReader.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace {

TEST(BagReaderTest, InvalidBagFile) {
  std::string invalid_bag_path = TEST_DATA_DIR "/invalid_bag.mcap";
  EXPECT_THROW(kalibr2::ros::BagImageReaderFactory::create(
                   invalid_bag_path, "/BFS_25037070/image"),
               std::runtime_error);
}

class BagReaderTestFixture : public ::testing::Test {
 protected:
  const std::string bag_path =
      TEST_DATA_DIR "/rosbag2_2025_06_11-12_00_21_0.mcap";
  const std::string topic = "/BFS_25037070/image";
  void SetUp() override {
    if (!std::filesystem::exists(bag_path)) {
      GTEST_SKIP() << "Bag file does not exist: " << bag_path;
    }
  }
};

TEST_F(BagReaderTestFixture, InvalidTopic) {
  EXPECT_THROW(
      kalibr2::ros::BagImageReaderFactory::create(bag_path, "/invalid_topic"),
      std::runtime_error);
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

  auto target_grid =
      boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(
          5, 6, 0.088, 0.2954);
  auto geometry =
      boost::make_shared<kalibr2::models::DistortedPinhole::Geometry>();
  auto detector = aslam::cameras::GridDetector(geometry, target_grid);

  while (reader->HasNext()) {
    kalibr2::Image img = reader->ReadNext();
    ASSERT_FALSE(img.image.empty());
    cv::imshow("Image", img.image);
    cv::waitKey(1);

    auto observation = kalibr2::ToObservation(img, detector);

    if (observation.has_value()) {
      std::cout << "Found target in image with timestamp: "
                << img.timestamp.toNSec() << std::endl;
    } else {
      std::cout << "No target found in image with timestamp: "
                << img.timestamp.toNSec() << std::endl;
    }
  }
}

TEST(BagReaderTest, Integration) {
  std::string bag_path =
      std::string(TEST_DATA_DIR) + "/rosbag2_2025_06_11-12_00_21_0.mcap";
  if (!std::filesystem::exists(bag_path)) {
    GTEST_SKIP() << "Bag file does not exist: " << bag_path;
  }
  auto reader =
      kalibr2::ros::create_bag_reader(bag_path, "/BFS_25037070/image");

  auto target_grid =
      boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(
          5, 6, 0.088, 0.2954);
  auto geometry =
      boost::make_shared<kalibr2::models::DistortedPinhole::Geometry>();
  auto detector = aslam::cameras::GridDetector(geometry, target_grid);

  size_t min_num_observation = 10;
  std::vector<aslam::cameras::GridCalibrationTargetObservation> observations;

  while (reader->HasNext()) {
    kalibr2::Image img = reader->ReadNext();
    assert(!img.image.empty());

    auto observation = kalibr2::ToObservation(img, detector);

    if (observation.has_value()) {
      observations.push_back(observation.value());
      std::cout << "Found target in image with timestamp: "
                << img.timestamp.toNSec() << std::endl;
      if (observations.size() >= min_num_observation) {
        std::cout << "Enough observations collected: " << observations.size()
                  << std::endl;
        break;  // Stop after collecting enough observations
      }
    } else {
      std::cout << "No target found in image with timestamp: "
                << img.timestamp.toNSec() << std::endl;
    }
  }

  double focal_lenght = 881.0;
  bool success = kalibr2::tools::CalibrateInstrinsics(observations, geometry,
                                                      detector, focal_lenght);

  ASSERT_TRUE(success) << "Failed to calibrate intrinsics from observations";
}

}  // namespace
