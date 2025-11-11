#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <boost/make_shared.hpp>
#include <sm/assert_macros.hpp>
#include <sm/logging.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgridv2.hpp>

namespace aslam {
namespace cameras {

/// \brief Construct an Aprilgrid calibration target
///        tagRows:    number of tags in y-dir (gridRows = 2*tagRows)
///        tagCols:    number of tags in x-dir (gridCols = 2*tagCols)
///        tagSize:    size of a tag [m]
///        tagSpacing: space between tags (in tagSpacing [m] = tagSpacing*tagSize)
///
///        corner ordering in _points :
///          12-----13  14-----15
///          | TAG 3 |  | TAG 4 |
///          8-------9  10-----11
///          4-------5  6-------7
///    y     | TAG 1 |  | TAG 2 |
///   ^      0-------1  2-------3
///   |-->x
GridCalibrationTargetAprilgridv2::GridCalibrationTargetAprilgridv2(
    size_t tagRows, size_t tagCols, double tagSize, double tagSpacing,
    const AprilgridOptionsv2 &options)
    : GridCalibrationTargetBase(2 * tagRows, 2 * tagCols),  //4  points per tag
      _tagSize(tagSize),
      _tagSpacing(tagSpacing),
      _options(options) {
  SM_ASSERT_GT(Exception, tagSize, 0.0, "tagSize has to be positive");
  SM_ASSERT_GT(Exception, tagSpacing, 0.0, "tagSpacing has to be positive");
  // Pre-allocate memory for the grid points
  _points.resize(size(), 3);

  // Initialize a normal grid (checkerboard and circlegrids)
  createGridPoints();
}

/// \brief initialize an april grid
///   point ordering: (e.g. 2x2 grid)
///          12-----13  14-----15
///          | TAG 3 |  | TAG 4 |
///          8-------9  10-----11
///          4-------5  6-------7
///    y     | TAG 1 |  | TAG 2 |
///   ^      0-------1  2-------3
///   |-->x
void GridCalibrationTargetAprilgridv2::createGridPoints() {
  for (unsigned r = 0; r < _rows; r++) {
    for (unsigned c = 0; c < _cols; c++) {
      Eigen::Matrix<double, 1, 3> point;

      point(0) = (int) (c / 2) * (1 + _tagSpacing) * _tagSize
          + (c % 2) * _tagSize;
      point(1) = (int) (r / 2) * (1 + _tagSpacing) * _tagSize
          + (r % 2) * _tagSize;
      point(2) = 0.0;

      _points.row(r * _cols + c) = point;
    }
  }
}


/// \brief extract the calibration target points from an image and write to an observation
bool GridCalibrationTargetAprilgridv2::computeObservation(
    const cv::Mat & image, Eigen::MatrixXd & outImagePoints,
    std::vector<bool> &outCornerObserved) const {

  bool success = true;

  // Detect the tags.
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::aruco::detectMarkers(image, _options.dictionary, markerCorners, markerIds, _options.detectorParameters, rejectedCandidates);
  // Aruco corner ordering per tag is
  ///          0-------1
  ///    y     |  TAG  |
  ///   ^      3-------2
  ///   |-->x
  // Check for minimum number of valid detected tags.
  auto validIds = std::count_if(markerIds.begin(), markerIds.end(),
                               [this](int id) {
                                 return id >= 0 && id < static_cast<int>(size() / 4);
                               });

  if (validIds < _options.minTagsForValidObs) {
    SM_DEBUG_STREAM(
      "Detected only " << validIds
      << " tags, which is less than the minimum required "
      << _options.minTagsForValidObs << " tags for a valid observation.\n"
    );
    return false;
  }

  // Order markers and markers corners by marker ID.
  std::vector<std::pair<int, std::vector<cv::Point2f>>> pairs;
  pairs.reserve(markerIds.size());
  for (size_t i = 0; i < markerIds.size(); ++i) pairs.emplace_back(markerIds[i], std::move(markerCorners[i]));
  std::sort(pairs.begin(), pairs.end(), [](auto &a, auto &b) { return a.first < b.first; });
  // unpack back
  for (size_t i = 0; i < pairs.size(); ++i) {
    markerIds[i] = pairs[i].first;
    markerCorners[i] = std::move(pairs[i].second);
  }



  // Reorder corners for each tag in counter-clockwise order starting from bottom-left.
  /// original (aruco's)
  ///          0-------1
  ///    y     |  TAG  |
  ///   ^      3-------2
  ///   |-->x
  /// new point ordering here
  ///          3-------2
  ///    y     | TAG 0 |
  ///   ^      0-------1
  ///   |-->x
  for (size_t i = 0; i < markerCorners.size(); ++i) {
    std::vector<cv::Point2f> originalCorners = markerCorners[i];
    markerCorners[i][0] = originalCorners[1];
    markerCorners[i][1] = originalCorners[0];
    markerCorners[i][2] = originalCorners[3];
    markerCorners[i][3] = originalCorners[2];
  }

  // Convert corners to cv::Mat (4 consecutive corners form one tag).
  cv::Mat tagCorners(4 * markerCorners.size(), 2, CV_32F);

  for (unsigned i = 0; i < markerCorners.size(); i++) {
    for (unsigned j = 0; j < 4; j++) {
      tagCorners.at<float>(4 * i + j, 0) = markerCorners[i][j].x;
      tagCorners.at<float>(4 * i + j, 1) = markerCorners[i][j].y;
    }
  }

  outCornerObserved.resize(size(), false);
  outImagePoints.resize(size(), 2);

  // Note: final ordering, both in the original and in v2 implementation
  // is different than as documented in the original kalibr code.
  // Example for a 2x2 tag grid:
  ///  |-- +x   1-------0 3-------2
  ///  |        | TAG 0 | | TAG 1 |
  ///  +        5-------4 7-------6
  ///  y        9-------8 11-----10
  ///           | TAG 2 | | TAG 3 |
  ///           13-----12 15-----14

  // Copy to outputs
  for (size_t i = 0; i < markerCorners.size(); ++i) {
    // get the tag id
    int tagId = markerIds[i];
    // skip tags with ids outside the valid range for this grid
    if (tagId < 0 || tagId >= static_cast<int>(size() / 4)) {
      SM_DEBUG_STREAM("Skipping tag with out-of-range ID: " << tagId << "\n");
      continue;
    }
    // compute the index of the first corner of this tag in the grid
    int baseId = (tagId / (_cols / 2)) * _cols * 2 + (tagId % (_cols / 2)) * 2;
    int pIdx[] = { baseId, baseId + 1, baseId + (int)_cols + 1, baseId + (int)_cols };
    // add four points per tag
    for (int j = 0; j < 4; j++) {
      double corner_x = tagCorners.row(4 * i + j).at<float>(0);
      double corner_y = tagCorners.row(4 * i + j).at<float>(1);
      outImagePoints.row(pIdx[j]) = Eigen::Matrix<double, 1, 2>(corner_x,
                                                                corner_y);
      // mark this corner as observed
      outCornerObserved[pIdx[j]] = true;
    }
  }
  return success;
}

}  // namespace cameras
}  // namespace aslam
