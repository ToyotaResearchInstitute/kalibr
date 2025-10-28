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
      int id = r * gridCols + c;
      cv::Mat marker;
      cv::aruco::drawMarker(dictionary, id, markerSize, marker, borderBits);

      int x = margin + c * (markerSize + spacing);
      int y = margin + r * (markerSize + spacing);
      marker.copyTo(canvas(cv::Rect(x, y, markerSize, markerSize)));
    }
  }

  return canvas;
}

// Validate that both versions of the AprilTag detector produce similar results
TEST (DetectorComparisonTest, Compare) {
  // 1. Generate test image with a 2x2 AprilTag grid and save it to disk
  constexpr int gridRows = 6;
  constexpr int gridCols = 5;
  constexpr int markerSize = 200;
  constexpr int margin = 40;
  constexpr int spacing = 40;
  constexpr int borderBits = 2;

  cv::Mat canvas = createTestAprilGrid(gridRows, gridCols, markerSize, margin, spacing, borderBits);

  // 2. Run with original kalibr AprilTag detector
  aslam::cameras::GridCalibrationTargetAprilgrid target1(gridRows, gridCols, 1, 1);

  Eigen::MatrixXd out1;
  std::vector<bool> obs1;

  auto t0 = std::chrono::high_resolution_clock::now();
  bool ok1 = target1.computeObservation(canvas, out1, obs1);
  auto t1 = std::chrono::high_resolution_clock::now();
  double ms1 = std::chrono::duration<double,std::milli>(t1-t0).count();
  std::cout << "v1 detection time: " << ms1 << " ms\n";
  ASSERT_TRUE(ok1) << "v1 detector failed to detect tags in synthetic image.";
  ASSERT_GT(out1.rows(), 0) << "v1 detector produced zero points matrix.";

  // 3. Run with new OpenCV AprilTag detector
  aslam::cameras::GridCalibrationTargetAprilgridv2 target2(gridRows, gridCols, 1, 1);

  Eigen::MatrixXd out2;
  std::vector<bool> obs2;

  auto t0_2 = std::chrono::high_resolution_clock::now();
  bool ok2 = target2.computeObservation(canvas, out2, obs2);
  auto t1_2 = std::chrono::high_resolution_clock::now();
  double ms2 = std::chrono::duration<double,std::milli>(t1_2-t0_2).count();
  std::cout << "v2 detection time: " << ms2 << " ms\n";
  ASSERT_TRUE(ok2) << "v2 detector failed to detect tags in synthetic image.";
  ASSERT_GT(out2.rows(), 0) << "v2 detector produced zero points matrix.";

  // Ensure both detectors produced the same number of points and observation flags
  ASSERT_EQ(out1.rows(), out2.rows()) << "v1 and v2 produced different number of points.";
  ASSERT_EQ(obs1.size(), obs2.size()) << "v1 and v2 observation flag vectors differ in size.";

  // Check per-corner positions within tolerance between versions
  constexpr double tolerance_px = 0.1;
  size_t numTags = out1.rows() / 4;
  for (size_t i = 0; i < numTags; ++i) {
    // skip tags not observed in both detectors
    if (!obs1.at(i * 4) || !obs2.at(i * 4)) continue;
    for (size_t j = 0; j < 4; ++j) {
      size_t idx = i * 4 + j;
      ASSERT_NEAR(out1.row(idx)(0), out2.row(idx)(0), tolerance_px) << "x mismatch tag " << i << " corner " << j;
      ASSERT_NEAR(out1.row(idx)(1), out2.row(idx)(1), tolerance_px) << "y mismatch tag " << i << " corner " << j;
    }
  }

}
