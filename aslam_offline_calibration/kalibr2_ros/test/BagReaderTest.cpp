#include <gtest/gtest.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <kalibr2/Image.hpp>
#include <kalibr2/CameraModels.hpp>
#include <kalibr2_ros/BagReader.hpp>

#include <aslam/cameras/GridDetector.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>


TEST(BagReaderTest, CanInitializeBagReader) {
  std::string bag_path = std::string(TEST_DATA_DIR) + "/rosbag2_2025_06_11-12_00_21_0.mcap";
  if (!std::filesystem::exists(bag_path)) {
    GTEST_SKIP() << "Bag file does not exist: " << bag_path;
  }
  auto reader = create_bag_reader(bag_path, "/BFS_25037070/image");
}


TEST(BagReaderTest, CanReadSingleImageSingleTopic) {
  std::string bag_path = std::string(TEST_DATA_DIR) + "/rosbag2_2025_06_11-12_00_21_0.mcap";
  if (!std::filesystem::exists(bag_path)) {
    GTEST_SKIP() << "Bag file does not exist: " << bag_path;
  }
  auto reader = create_bag_reader(bag_path, "/BFS_25037070/image");
  Image img = reader->ReadNext();
}


TEST(DISABLED_BagReaderTest, CanReadMultipleImagesSingleTopic) {
  std::string bag_path = std::string(TEST_DATA_DIR) + "/rosbag2_2025_06_11-12_00_21_0.mcap";
  if (!std::filesystem::exists(bag_path)) {
    GTEST_SKIP() << "Bag file does not exist: " << bag_path;
  }
  auto reader = create_bag_reader(bag_path, "/BFS_25037070/image");
  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

  while (reader->HasNext()) {
    Image img = reader->ReadNext();
    assert (!img.image.empty());
    cv::imshow("Image", img.image);
    cv::waitKey(1);
  }
}


TEST(DISABLED_BagReaderTest, CanReadMultipleImagesMultipleTopics)
{
  std::string bag_path = std::string(TEST_DATA_DIR) + "/rosbag2_2025_06_11-12_00_21_0.mcap";
  if (!std::filesystem::exists(bag_path)) {
    GTEST_SKIP() << "Bag file does not exist: " << bag_path;
  }
  std::array<std::string, 2> topics = {"/BFS_25037070/image", "/BFS_24293899/image"};
  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

  for (const auto& topic : topics) {
    auto reader = create_bag_reader(bag_path, topic);
    while (reader->HasNext()) {
      Image img = reader->ReadNext();
      assert (!img.image.empty());
      cv::imshow("Image", img.image);
      cv::waitKey(1);
    }
  }
}


TEST(DISABLED_BagReaderTest, CanDetectMultipleImagesSingleTopic) {
  std::string bag_path = std::string(TEST_DATA_DIR) + "/rosbag2_2025_06_11-12_00_21_0.mcap";
  if (!std::filesystem::exists(bag_path)) {
    GTEST_SKIP() << "Bag file does not exist: " << bag_path;
  }
  auto reader = create_bag_reader(bag_path, "/BFS_25037070/image");
  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

  auto target_grid = boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(5, 6, 0.088, 0.2954);
  auto geometry = boost::make_shared<kalibr2::models::DistortedPinhole::Geometry>();
  auto detector = aslam::cameras::GridDetector(geometry, target_grid);

  while (reader->HasNext()) {
    Image img = reader->ReadNext();
    assert (!img.image.empty());
    cv::imshow("Image", img.image);
    cv::waitKey(1);

    auto observation = ToObservation(img, detector);

    if (observation.has_value()) {
      std::cout << "Found target in image with timestamp: " << img.timestamp.toNSec() << std::endl;
    }
    else {
      std::cout << "No target found in image with timestamp: " << img.timestamp.toNSec() << std::endl;
    }
  }
}
