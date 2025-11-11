#include <iostream>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgridv2.hpp>
#include <gtest/gtest.h>

using namespace std;

// Create a synthetic AprilTag grid image (grayscale) for tests.
// Parameters use pixels for sizes and spacing. Returns a CV_8UC1 image.
cv::Mat createTestAprilGrid(int gridRows,
                            int gridCols,
                            int markerSize = 200,
                            int margin = 40,
                            int spacing = 40,
                            int borderBits = 2) {

  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

  const int canvasWidth = margin * 2 + gridCols * (markerSize) + (gridCols - 1) * spacing;
  const int canvasHeight = margin * 2 + gridRows * (markerSize) + (gridRows - 1) * spacing;

  // white background
  cv::Mat canvas(canvasHeight, canvasWidth, CV_8UC1, cv::Scalar(255));

  // draw markers with ids sequential left-to-right, top-to-bottom
  for (int r = 0; r < gridRows; ++r) {
    for (int c = 0; c < gridCols; ++c) {
      const int id = r * gridCols + c;
      cv::Mat marker;
      cv::aruco::drawMarker(dictionary, id, markerSize, marker, borderBits);

      const int x = margin + c * (markerSize + spacing);
      const int y = margin + r * (markerSize + spacing);
      marker.copyTo(canvas(cv::Rect(x, y, markerSize, markerSize)));
    }
  }

  return canvas;
}

// Validate that both versions of the AprilTag detector produce similar results
TEST (DetectorComparisonTest, Compare) {
  // 1. Generate test image with a 2x2 AprilTag grid and save it to disk
  constexpr int kGridRows = 6;
  constexpr int kGridCols = 5;
  constexpr int kMarkerSize = 200;
  constexpr int kMargin = 40;
  constexpr int kSpacing = 40;
  constexpr int kBorderBits = 2;

  cv::Mat canvas = createTestAprilGrid(kGridRows, kGridCols, kMarkerSize, kMargin, kSpacing, kBorderBits);

  // 2. Run with original kalibr AprilTag detector
  aslam::cameras::GridCalibrationTargetAprilgrid target1(kGridRows, kGridCols, 1, 1);

  Eigen::MatrixXd output_1;
  std::vector<bool> observation_detected_1;

  const auto t0 = std::chrono::high_resolution_clock::now();
  const bool ok1 = target1.computeObservation(canvas, output_1, observation_detected_1);
  const auto t1 = std::chrono::high_resolution_clock::now();
  const double ms1 = std::chrono::duration<double,std::milli>(t1-t0).count();
  std::cout << "v1 detection time: " << ms1 << " ms\n";
  ASSERT_TRUE(ok1) << "v1 detector failed to detect tags in synthetic image.";
  ASSERT_GT(output_1.rows(), 0) << "v1 detector produced zero points matrix.";

  // 3. Run with new OpenCV AprilTag detector
  aslam::cameras::GridCalibrationTargetAprilgridv2 target2(kGridRows, kGridCols, 1, 1);

  Eigen::MatrixXd output_2;
  std::vector<bool> observation_detected_2;

  const auto t0_2 = std::chrono::high_resolution_clock::now();
  const bool ok2 = target2.computeObservation(canvas, output_2, observation_detected_2);
  const auto t1_2 = std::chrono::high_resolution_clock::now();
  const double ms2 = std::chrono::duration<double,std::milli>(t1_2-t0_2).count();
  std::cout << "v2 detection time: " << ms2 << " ms\n";
  ASSERT_TRUE(ok2) << "v2 detector failed to detect tags in synthetic image.";
  ASSERT_GT(output_2.rows(), 0) << "v2 detector produced zero points matrix.";

  // Ensure both detectors produced the same number of points and observation flags
  ASSERT_EQ(output_1.rows(), output_2.rows()) << "v1 and v2 produced different number of points.";
  ASSERT_EQ(observation_detected_1.size(), observation_detected_2.size()) << "v1 and v2 observation flag vectors differ in size.";

  // Check per-corner positions within tolerance between versions
  constexpr double tolerance_px = 0.1;
  const size_t numTags = output_1.rows() / 4;
  for (size_t i = 0; i < numTags; ++i) {
    // skip tags not observed in both detectors
    if (!observation_detected_1.at(i * 4) || !observation_detected_2.at(i * 4)) continue;
    for (size_t j = 0; j < 4; ++j) {
      const size_t idx = i * 4 + j;
      ASSERT_NEAR(output_1.row(idx)(0), output_2.row(idx)(0), tolerance_px) << "x mismatch tag " << i << " corner " << j;
      ASSERT_NEAR(output_1.row(idx)(1), output_2.row(idx)(1), tolerance_px) << "y mismatch tag " << i << " corner " << j;
    }
  }

}
