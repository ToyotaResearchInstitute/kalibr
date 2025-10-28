#pragma once

#include <vector>

#include <aslam/cameras/GridCalibrationTargetBase.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <sm/assert_macros.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>


namespace aslam {
namespace cameras {

/// \brief Aprilgrid calibration target using OpenCV's ArUco module
// This class is based on the original GridCalibrationTargetAprilgrid,
// but uses OpenCV's ArUco module for AprilTag detection instead of the
// original ethz apriltag library.

// Which speeds up detection significantly (~x10-x40)

class GridCalibrationTargetAprilgridv2 : public GridCalibrationTargetBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  typedef boost::shared_ptr<GridCalibrationTargetAprilgridv2> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetAprilgridv2> ConstPtr;

  // target extraction options
  // Relies on OpenCV's ArUco DetectorParameters
  struct AprilgridOptionsv2 {
    // Default constructor with default options
    AprilgridOptionsv2() :
      detectorParameters(cv::aruco::DetectorParameters::create()),
      dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)),
      minTagsForValidObs(4) {
        detectorParameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detectorParameters->markerBorderBits = 2;
      }
    ~AprilgridOptionsv2() {}

    //options
    cv::Ptr<cv::aruco::DetectorParameters> detectorParameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    /// \brief min. number of tags for a valid observation
    unsigned int minTagsForValidObs;
  };

  /// \brief initialize based on checkerboard geometry
  GridCalibrationTargetAprilgridv2(size_t tagRows, size_t tagCols, double tagSize,
                                 double tagSpacing, const AprilgridOptionsv2 &options = AprilgridOptionsv2());

  virtual ~GridCalibrationTargetAprilgridv2() {};

  /// \brief initialize the grid with the points
  void createGridPoints();

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat & image,
                          Eigen::MatrixXd & outImagePoints,
                          std::vector<bool> &outCornerObserved) const;

 private:

  /// \brief size of a tag [m]
  double _tagSize;

  /// \brief space between tags (tagSpacing [m] = tagSize * tagSpacing)
  double _tagSpacing;

  /// \brief target extraction options
  AprilgridOptionsv2 _options;
};


}  // namespace cameras
}  // namespace aslam
