#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <array>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sm/kinematics/Transformation.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>

namespace kalibr2 {
namespace ros {
namespace qa {

enum class QaErrorCode : int32_t {
  ACCEPTED = 0,
  NO_DETECTION = 1,
  OUT_OF_BOUNDS = 2,
  BLURRY = 3,
  ALREADY_COMPLETE = 4,
  NO_VARIANCE = 5,
  POSE_ESTIMATION_FAILED = 6,
  NO_DISTANCE_VARIANCE = 7,
  NO_ANGLE_VARIANCE = 8
};

struct BinState {
  int accepted_frames = 0;
  std::vector<double> tilt_angles;
  std::vector<double> distances;
  int last_point_count = 0;

  const double MIN_ANGLE_VARIANCE = 0.15; // radians (~20 degrees)
  const double MIN_DIST_VARIANCE  = 0;  // meters

  bool isComplete() const {
    if (accepted_frames < 2) return false;

    double min_d = *std::min_element(distances.begin(), distances.end());
    double max_d = *std::max_element(distances.begin(), distances.end());
    if ((max_d - min_d) < MIN_DIST_VARIANCE) return false;

    double min_a = *std::min_element(tilt_angles.begin(), tilt_angles.end());
    double max_a = *std::max_element(tilt_angles.begin(), tilt_angles.end());
    if ((max_a - min_a) < MIN_ANGLE_VARIANCE) return false;

    return true;
  }
};

class DataQualityTracker {
 public:
  DataQualityTracker(int img_width, int img_height, int grid_cols = 2, int grid_rows = 2)
      : img_width_(img_width),
        img_height_(img_height),
        grid_cols_(grid_cols),
        grid_rows_(grid_rows) {
    
    bins_.resize(grid_cols_ * grid_rows_);

    // Initialize ROS node
    node_ = rclcpp::Node::make_shared("kalibr_qa_monitor");
    heatmap_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("~/heatmap", 10);
    completeness_pub_ = node_->create_publisher<std_msgs::msg::Float32>("~/completeness", 10);
    status_pub_ = node_->create_publisher<std_msgs::msg::Int32>("~/qa_status", 10);
  }

  ~DataQualityTracker() {
    LogOutcomeSummary();
  }

  void publishStatus(QaErrorCode code) {
    std_msgs::msg::Int32 msg;
    msg.data = static_cast<int32_t>(code);
    status_pub_->publish(msg);
  }

  void RecordOutcome(QaErrorCode code, bool publish_status = true) {
    const size_t idx = static_cast<size_t>(code);
    if (idx < outcome_counts_.size()) {
      outcome_counts_[idx]++;
    }
    if (code != QaErrorCode::ACCEPTED) {
      rejected_frames_++;
    }
    total_frames_++;

    if (publish_status) {
      publishStatus(code);
    }
  }

  void LogOutcomeSummary() const {
    if (!node_ || total_frames_ == 0) {
      return;
    }

    RCLCPP_INFO(node_->get_logger(),
                "QA outcome summary: total=%zu, accepted=%zu, rejected=%zu",
                total_frames_,
                outcome_counts_[static_cast<size_t>(QaErrorCode::ACCEPTED)],
                rejected_frames_);

    RCLCPP_INFO(node_->get_logger(), "Rejected breakdown:");
    RCLCPP_INFO(node_->get_logger(), "  NO_DETECTION: %zu",
                outcome_counts_[static_cast<size_t>(QaErrorCode::NO_DETECTION)]);
    RCLCPP_INFO(node_->get_logger(), "  OUT_OF_BOUNDS: %zu",
                outcome_counts_[static_cast<size_t>(QaErrorCode::OUT_OF_BOUNDS)]);
    RCLCPP_INFO(node_->get_logger(), "  BLURRY: %zu",
                outcome_counts_[static_cast<size_t>(QaErrorCode::BLURRY)]);
    RCLCPP_INFO(node_->get_logger(), "  ALREADY_COMPLETE: %zu",
                outcome_counts_[static_cast<size_t>(QaErrorCode::ALREADY_COMPLETE)]);
    RCLCPP_INFO(node_->get_logger(), "  NO_VARIANCE: %zu",
                outcome_counts_[static_cast<size_t>(QaErrorCode::NO_VARIANCE)]);
    RCLCPP_INFO(node_->get_logger(), "  POSE_ESTIMATION_FAILED: %zu",
                outcome_counts_[static_cast<size_t>(QaErrorCode::POSE_ESTIMATION_FAILED)]);
    RCLCPP_INFO(node_->get_logger(), "  NO_DISTANCE_VARIANCE: %zu",
                outcome_counts_[static_cast<size_t>(QaErrorCode::NO_DISTANCE_VARIANCE)]);
    RCLCPP_INFO(node_->get_logger(), "  NO_ANGLE_VARIANCE: %zu",
                outcome_counts_[static_cast<size_t>(QaErrorCode::NO_ANGLE_VARIANCE)]);
  }

  bool evaluateFrameQuality(const cv::Mat& img,
                            const sm::kinematics::Transformation& T_t_c,
                            const aslam::cameras::GridCalibrationTargetObservation& obs,
                            QaErrorCode& out_code) {
    std::vector<cv::Point2f> corners_image_frame;
    unsigned int num_corners = obs.getCornersImageFrame(corners_image_frame);
    if (num_corners == 0) {
      out_code = QaErrorCode::NO_DETECTION;
      RecordOutcome(out_code);
      return false;
    }

    constexpr int kAprilgridRows = 6;
    constexpr int kAprilgridCols = 6;
    const int total_points = kAprilgridRows * kAprilgridCols * 4;
    const int min_points = static_cast<int>(std::ceil(total_points * 0.50));
    if (static_cast<int>(num_corners) < min_points) {
      out_code = QaErrorCode::NO_DETECTION;
      RecordOutcome(out_code);
      return false;
    }

    // 1. Calculate Motion Blur (Laplacian Variance) on the detected crop
    cv::Rect roi = cv::boundingRect(corners_image_frame);
    roi &= cv::Rect(0, 0, img.cols, img.rows); // ensure it stays within image bounds
    if (roi.width == 0 || roi.height == 0) {
      out_code = QaErrorCode::NO_DETECTION;
      RecordOutcome(out_code);
      return false;
    }

    cv::Mat cropped_img = img(roi);
    cv::Mat gray, laplacian;
    if (cropped_img.channels() == 3) {
      cv::cvtColor(cropped_img, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = cropped_img;
    }
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    double variance = stddev.val[0] * stddev.val[0];

    //1000 is crystal clear
    const double BLUR_THRESHOLD = 600.0; // Lowered to 50 for testing
    if (variance < BLUR_THRESHOLD) {
      out_code = QaErrorCode::BLURRY;
      RecordOutcome(out_code);
      return false; // Too blurry
    }

    // 2. Find Grid Index
    int grid_idx = getDetectionGridIndex(obs);
    if (grid_idx < 0 || grid_idx >= (int)bins_.size()) {
      out_code = QaErrorCode::OUT_OF_BOUNDS;
      RecordOutcome(out_code);
      return false; // Not found or out of bounds
    }

    BinState& current_bin = bins_[grid_idx];

    // 3. Extract Geometry
    double distance = T_t_c.T()(2); // Z-axis translation
    // T_t_c.C() is the rotation matrix. (2, 2) is the Z-Z dot product (angle to optical axis).
    double tilt_angle = std::acos(std::clamp(T_t_c.C()(2, 2), -1.0, 1.0));

    // 4. Check for minimum spatial difference from last frame to avoid redundancies
    if (current_bin.accepted_frames > 0) {
      bool is_new_distance = std::abs(current_bin.distances.back() - distance) > 0.05;
      bool is_new_angle = std::abs(current_bin.tilt_angles.back() - tilt_angle) > 0.1;
      if (!is_new_distance) {
        if (num_corners > static_cast<unsigned int>(current_bin.last_point_count) && !is_new_angle) {
          current_bin.distances.back() = distance;
          current_bin.tilt_angles.back() = tilt_angle;
          current_bin.last_point_count = static_cast<int>(num_corners);
          out_code = QaErrorCode::ACCEPTED;
          RecordOutcome(out_code);
          publishMetrics();
          return true;
        }
        out_code = QaErrorCode::NO_DISTANCE_VARIANCE;
        RecordOutcome(out_code);
        return false;
      }
      if (!is_new_angle) {
        if (num_corners > static_cast<unsigned int>(current_bin.last_point_count) && !is_new_distance) {
          current_bin.distances.back() = distance;
          current_bin.tilt_angles.back() = tilt_angle;
          current_bin.last_point_count = static_cast<int>(num_corners);
          out_code = QaErrorCode::ACCEPTED;
          RecordOutcome(out_code);
          publishMetrics();
          return true;
        }
        out_code = QaErrorCode::NO_ANGLE_VARIANCE;
        RecordOutcome(out_code);
        return false;
      }
    }

    // Passed QA!
    current_bin.distances.push_back(distance);
    current_bin.tilt_angles.push_back(tilt_angle);
    current_bin.accepted_frames++;
    current_bin.last_point_count = static_cast<int>(num_corners);

    out_code = QaErrorCode::ACCEPTED;
    RecordOutcome(out_code);
    publishMetrics();
    return true;
  }

  cv::Mat getHeatmapImage() const {
    cv::Mat heatmap = cv::Mat::zeros(grid_rows_, grid_cols_, CV_8UC1);
    for (int r = 0; r < grid_rows_; ++r) {
      for (int c = 0; c < grid_cols_; ++c) {
        int idx = r * grid_cols_ + c;
        int frames = bins_[idx].accepted_frames;
        int intensity = std::min(255, std::min(frames, 8) * 32);
        heatmap.at<uint8_t>(r, c) = intensity;
      }
    }
    cv::Mat color_heatmap;
    cv::applyColorMap(heatmap, color_heatmap, cv::COLORMAP_JET);
    cv::Mat render;
    cv::resize(color_heatmap, render, cv::Size(grid_cols_ * 50, grid_rows_ * 50), 0, 0, cv::INTER_NEAREST);
    return render;
  }

  void spinROSOnce() {
    rclcpp::spin_some(node_);
  }

  bool isGlobalCoverageComplete() const {
    return std::all_of(bins_.begin(), bins_.end(), [](const BinState& b) { return b.isComplete(); });
  }

 private:
  int getDetectionGridIndex(const aslam::cameras::GridCalibrationTargetObservation& obs) const {
    std::vector<cv::Point2f> corners_image_frame;
    unsigned int num_corners = obs.getCornersImageFrame(corners_image_frame);
    
    if (num_corners == 0) return -1;

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    for (const auto& pt : corners_image_frame) {
      sum_x += pt.x;
      sum_y += pt.y;
    }
    float centroid_x = sum_x / num_corners;
    float centroid_y = sum_y / num_corners;

    float cell_width = static_cast<float>(img_width_) / grid_cols_;
    float cell_height = static_cast<float>(img_height_) / grid_rows_;

    int grid_x = std::clamp(static_cast<int>(centroid_x / cell_width), 0, grid_cols_ - 1);
    int grid_y = std::clamp(static_cast<int>(centroid_y / cell_height), 0, grid_rows_ - 1);

    return (grid_y * grid_cols_) + grid_x;
  }

  void publishMetrics() {
    int complete_bins = 0;
    for (const auto& b : bins_) {
      if (b.isComplete()) {
        complete_bins++;
      }
    }
    float completeness = (static_cast<float>(complete_bins) / bins_.size()) * 100.0f;

    std_msgs::msg::Float32 completeness_msg;
    completeness_msg.data = completeness;
    completeness_pub_->publish(completeness_msg);

    // Render Heatmap (small internal size)
    cv::Mat heatmap = cv::Mat::zeros(grid_rows_, grid_cols_, CV_8UC1);
    for (int r = 0; r < grid_rows_; ++r) {
      for (int c = 0; c < grid_cols_; ++c) {
        int idx = r * grid_cols_ + c;
        int frames = bins_[idx].accepted_frames;
        int intensity = std::min(255, frames * 80); // Cap at 255 (3 frames = ~240/hot color)
        heatmap.at<uint8_t>(r, c) = intensity;
      }
    }

    cv::Mat color_heatmap;
    cv::applyColorMap(heatmap, color_heatmap, cv::COLORMAP_JET);

    // Upscale to match image aspect ratio for RViz viewing
    cv::Mat render;
    cv::resize(color_heatmap, render, cv::Size(grid_cols_ * 50, grid_rows_ * 50), 0, 0, cv::INTER_NEAREST);

    sensor_msgs::msg::Image::SharedPtr ros_img = 
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", render).toImageMsg();
    heatmap_pub_->publish(*ros_img);
  }

  int img_width_;
  int img_height_;
  int grid_cols_;
  int grid_rows_;
  std::vector<BinState> bins_;
  std::array<size_t, 9> outcome_counts_{};
  size_t total_frames_ = 0;
  size_t rejected_frames_ = 0;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr heatmap_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr completeness_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
};

} // namespace qa
} // namespace ros
} // namespace kalibr2