
#include <aslam/cameras.hpp>
#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <gtest/gtest.h>
#include <kalibr2/CameraModels.hpp>
#include <kalibr2/Image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace {

TEST(ImageTest, ToObservationNoTarget) {
  // Black image emulating no target
  kalibr2::Image img;
  img.timestamp = aslam::Time(1622547800, 0);
  img.image = cv::Mat::zeros(480, 640, CV_8UC3);

  auto target_grid =
      boost::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(5, 7,
                                                                          0.01);
  auto geometry =
      boost::make_shared<kalibr2::models::DistortedPinhole::Geometry>();
  auto detector = aslam::cameras::GridDetector(geometry, target_grid);

  auto observation = kalibr2::ToObservation(img, detector);

  ASSERT_FALSE(observation.has_value());
}

TEST(ImageTest, ToObservationCircleGridTarget) {
  kalibr2::Image img;
  img.timestamp = aslam::Time(1622547800, 0);
  img.image = cv::imread("data/testImageCircleGrid.jpg", cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(img.image.empty());

  auto target_grid =
      boost::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(5, 7,
                                                                          0.01);
  auto geometry =
      boost::make_shared<kalibr2::models::DistortedPinhole::Geometry>();
  auto detector = aslam::cameras::GridDetector(geometry, target_grid);

  auto observation = kalibr2::ToObservation(img, detector);

  ASSERT_TRUE(observation.has_value());
}

}  // namespace
