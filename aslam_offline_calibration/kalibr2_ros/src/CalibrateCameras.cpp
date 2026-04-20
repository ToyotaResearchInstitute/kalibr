#include <CLI/CLI.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgridv2.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <kalibr2/CalibrationTools.hpp>
#include <kalibr2/CameraCalibrator.hpp>
#include <kalibr2/CameraGraph.hpp>
#include <kalibr2/CameraModels.hpp>
#include <kalibr2/Image.hpp>
#include <kalibr2/SynchronizedObservationView.hpp>
#include <kalibr2_ros/BagReader.hpp>
#include <kalibr2_ros/Config.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <limits>

// Temporal includes
#include <aslam/calibration/core/IncrementalEstimator.h>
#include <kalibr2_ros/ROSToYAMLConverter.hpp>
#include <kalibr2_ros/QualityAssurance.hpp> // <-- Added our QA header

using kalibr2::ros::CalibrationConfig;
using kalibr2::ros::CameraConfig;

struct FrameObservation {
  aslam::cameras::GridCalibrationTargetObservation observation;
  cv::Mat image;
};

//our QA node 



//MARKER KUARTIS
//MODIFY THIS TO OBTAIN OBSERVATIONS, AND THE POSE SIMULTANEOUSLY SO THAT DATA QUALITY CAN BE MEASURED
std::vector<FrameObservation> get_observations_from_camera(
    kalibr2::ImageReader& reader, const aslam::cameras::GridDetector& detector, std::optional<size_t> max_observations,
  const std::string& camera_name, size_t frame_stride) {
  std::vector<FrameObservation> frame_observations;
  const size_t message_count = reader.MessageCount();
  size_t images_processed = 0;
  bool target_notice_printed = false;

  std::cout << "[" << camera_name << "] Starting observation extraction from " << message_count << " images";
  if (max_observations.has_value()) {
    std::cout << " (target: " << max_observations.value() << " observations)";
  }
  std::cout << std::endl;
  //PIPELINE::THE IMAGE IS BEING READ HERE 
  while (reader.HasNext()) {
    kalibr2::Image img = reader.ReadNext();
    images_processed++;

    // Only process every Nth frame to reduce CPU/RAM pressure.
    if (((images_processed - 1) % frame_stride) != 0) {
      std::cout << "\r[" << camera_name << "] Progress: " << images_processed << "/" << message_count << " images, "
                << frame_observations.size() << " observations ("
                << (images_processed > 0 ? (100.0 * frame_observations.size() / images_processed) : 0.0) << "% detected)"
                << std::flush;
      continue;
    }

    auto observation = kalibr2::ToObservation(img, detector);

    if (observation.has_value()) {
      frame_observations.push_back({observation.value(), img.image.clone()});

      if (max_observations.has_value() &&
          !target_notice_printed &&
          frame_observations.size() >= max_observations.value()) {
        std::cout << std::endl;
        std::cout << "[" << camera_name << "] Reached target of " << max_observations.value()
                  << " observations after processing " << images_processed
                  << " images, but continuing to scan all frames for QA." << std::endl;
        target_notice_printed = true;
      }
    }

    std::cout << "\r[" << camera_name << "] Progress: " << images_processed << "/" << message_count << " images, "
              << frame_observations.size() << " observations ("
              << (images_processed > 0 ? (100.0 * frame_observations.size() / images_processed) : 0.0) << "% detected)"
              << std::flush;
  }

  std::cout << std::endl;
  std::cout << "[" << camera_name << "] Final: " << frame_observations.size() << " observations from " << images_processed
            << " images (" << (images_processed > 0 ? (100.0 * frame_observations.size() / images_processed) : 0.0)
            << "% detection rate)" << std::endl;

  return frame_observations;
}


// QA Tracker Function to find grid index
int getDetectionGridIndex(const aslam::cameras::GridCalibrationTargetObservation& obs, 
                          int image_width, int image_height, int grid_cols = 6, int grid_rows = 5) {
  std::vector<cv::Point2f> corners_image_frame;
  unsigned int num_corners = obs.getCornersImageFrame(corners_image_frame);
  
  if (num_corners == 0) {
      return -1; // No target detected
  }

  float sum_x = 0.0f;
  float sum_y = 0.0f;
  for (const auto& pt : corners_image_frame) {
      sum_x += pt.x;
      sum_y += pt.y;
  }
  float centroid_x = sum_x / num_corners;
  float centroid_y = sum_y / num_corners;

  float cell_width = static_cast<float>(image_width) / grid_cols;
  float cell_height = static_cast<float>(image_height) / grid_rows;

  int grid_x = std::max(0, std::min(static_cast<int>(centroid_x / cell_width), grid_cols - 1));
  int grid_y = std::max(0, std::min(static_cast<int>(centroid_y / cell_height), grid_rows - 1));

  return (grid_y * grid_cols) + grid_x;
}

std::string qaErrorCodeToString(kalibr2::ros::qa::QaErrorCode code) {
  switch (code) {
    case kalibr2::ros::qa::QaErrorCode::ACCEPTED: return "ACCEPTED";
    case kalibr2::ros::qa::QaErrorCode::NO_DETECTION: return "NO DETECTION";
    case kalibr2::ros::qa::QaErrorCode::OUT_OF_BOUNDS: return "OUT OF BOUNDS";
    case kalibr2::ros::qa::QaErrorCode::BLURRY: return "BLURRY";
    case kalibr2::ros::qa::QaErrorCode::ALREADY_COMPLETE: return "ALREADY COMPLETE";
    case kalibr2::ros::qa::QaErrorCode::NO_VARIANCE: return "NO VARIANCE";
    case kalibr2::ros::qa::QaErrorCode::POSE_ESTIMATION_FAILED: return "POSE EST FAILED";
    case kalibr2::ros::qa::QaErrorCode::NO_DISTANCE_VARIANCE: return "NO DIST VARIANCE";
    case kalibr2::ros::qa::QaErrorCode::NO_ANGLE_VARIANCE: return "NO ANGLE VARIANCE";
    default: return "UNKNOWN";
  }
}

std::vector<aslam::cameras::GridCalibrationTargetObservation> get_observations_from_camera_with_calib_QA(
    kalibr2::ImageReader& reader, const aslam::cameras::GridDetector& detector,
    std::optional<size_t> max_observations,
  const std::string& camera_name, size_t frame_stride) {
  std::vector<aslam::cameras::GridCalibrationTargetObservation> observations;
  const size_t message_count = reader.MessageCount();
  size_t images_processed = 0;
  bool target_notice_printed = false;

  // Initialize DataQualityTracker later once we read the first image
  std::unique_ptr<kalibr2::ros::qa::DataQualityTracker> qa_tracker;

  std::string window_detections = "Detections QA - " + camera_name;
  std::string window_heatmap = "Heatmap - " + camera_name;
  cv::namedWindow(window_detections, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(window_heatmap, cv::WINDOW_AUTOSIZE);

  std::cout << "[" << camera_name << "] Starting custom QA observation extraction from " << message_count << " images";
  if (max_observations.has_value()) {
    std::cout << " (target: " << max_observations.value() << " observations)";
  }
  std::cout << std::endl;

  //PIPELINE::THE IMAGE IS BEING READ HERE 
  while (reader.HasNext()) {
    kalibr2::Image img = reader.ReadNext();
    images_processed++;

    // Only process every Nth frame to reduce CPU/RAM pressure.
    if (((images_processed - 1) % frame_stride) != 0) {
      std::cout << "\r[" << camera_name << "] Progress: " << images_processed << "/" << message_count << " images, "
                << observations.size() << " observations ("
                << (images_processed > 0 ? (100.0 * observations.size() / images_processed) : 0.0) << "% detected)"
                << std::flush;
      continue;
    }

    if (!qa_tracker) {
      qa_tracker = std::make_unique<kalibr2::ros::qa::DataQualityTracker>(img.image.cols, img.image.rows, 4, 3);
    }

    auto observation = kalibr2::ToObservation(img, detector);
    
    cv::Mat display_img = img.image.clone();
    if (display_img.channels() == 1) {
      cv::cvtColor(display_img, display_img, cv::COLOR_GRAY2BGR);
    }
    
    kalibr2::ros::qa::QaErrorCode status_code = kalibr2::ros::qa::QaErrorCode::NO_DETECTION;

    if (observation.has_value()) {
      // Just extract points for visualizing what was detected
      std::vector<cv::Point2f> corners;
      observation.value().getCornersImageFrame(corners);
      for(size_t i = 0; i < corners.size(); i++){
        cv::circle(display_img, corners[i], 3, cv::Scalar(0, 255, 0), -1);
      }
      
      status_code = kalibr2::ros::qa::QaErrorCode::ACCEPTED;

      // In the new pipeline, we just gather observations blindly here.
      // QA filtering happens later during CalibrateSingleCameraQA when poses are safe.
      observations.push_back(observation.value());
    }

    std::cout << "\r[" << camera_name << "] Progress: " << images_processed << "/" << message_count << " images, "
              << observations.size() << " observations ("
              << (images_processed > 0 ? (100.0 * observations.size() / images_processed) : 0.0) << "% detected)"
              << std::flush;

    // GUI Interaction
    cv::Scalar text_color = (status_code == kalibr2::ros::qa::QaErrorCode::ACCEPTED) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::putText(display_img, qaErrorCodeToString(status_code), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);

    if (status_code == kalibr2::ros::qa::QaErrorCode::ACCEPTED) {
         cv::putText(display_img, "Target Saved", cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow(window_detections, display_img);
    if (qa_tracker) {
      cv::imshow(window_heatmap, qa_tracker->getHeatmapImage());
      qa_tracker->spinROSOnce(); // ensure our topics broadcast!
    }
    cv::waitKey(1); // Wait 1ms so OpenCV can draw
    
    if (max_observations.has_value() &&
        !target_notice_printed &&
        observations.size() >= max_observations.value()) {
      std::cout << std::endl;
      std::cout << "[" << camera_name << "] Reached target of " << max_observations.value()
                << " observations after processing " << images_processed
                << " images, but continuing to scan all frames for QA." << std::endl;
      target_notice_printed = true;
    }
  }

  std::cout << std::endl;
  std::cout << "[" << camera_name << "] Final: " << observations.size() << " observations from " << images_processed
            << " images (" << (images_processed > 0 ? (100.0 * observations.size() / images_processed) : 0.0)
            << "% detection rate)" << std::endl;

  cv::destroyWindow(window_detections);
  cv::destroyWindow(window_heatmap);
  
  return observations;
}

bool CalibrateSingleCameraQA(std::vector<FrameObservation>& frame_observations,
                             const boost::shared_ptr<kalibr2::CameraCalibratorBase>& camera_calibrator,
                             const aslam::cameras::GridCalibrationTargetBase::Ptr& target,
                             std::optional<double> fallback_focal_length,
                             const std::string& camera_name) {
  
  if (frame_observations.empty()) {
    SM_WARN("No observations for camera %s", camera_name.c_str());
    return false;
  }

  std::vector<aslam::cameras::GridCalibrationTargetObservation> observations;
  observations.reserve(frame_observations.size());
  for (const auto& frame_observation : frame_observations) {
    observations.push_back(frame_observation.observation);
  }

  int img_cols = observations[0].imCols();
  int img_rows = observations[0].imRows();

  // First initialize intrinsics exactly like the Kalibr pipeline 
  // (which is safe because it has all views now). 
  bool success = camera_calibrator->camera_geometry()->initializeIntrinsics(observations, fallback_focal_length);
  if (!success) {
    SM_WARN("Failed to initialize intrinsics for camera %s", camera_name.c_str());
    return false;
  }

  auto problem = boost::make_shared<aslam::calibration::OptimizationProblem>();
  camera_calibrator->AddIntrinsicDesignVariables(problem);

  constexpr double corner_uncertainty = 1.0;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * corner_uncertainty * corner_uncertainty;
  Eigen::Matrix2d invR = R.inverse();

  // Setup QA GUI
  std::unique_ptr<kalibr2::ros::qa::DataQualityTracker> qa_tracker = 
    std::make_unique<kalibr2::ros::qa::DataQualityTracker>(img_cols, img_rows, 4, 3);
  std::string window_name = "QA Post-Extraction Pose - " + camera_name;
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

  std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>> target_pose_dvs;
  std::vector<FrameObservation> valid_frame_observations;
  valid_frame_observations.reserve(frame_observations.size());

  std::cout << "[" << camera_name << "] Applying QA tracker after intrinsic initialization..." << std::endl;

  for (size_t i = 0; i < frame_observations.size(); ++i) {
    const auto& frame_observation = frame_observations[i];
    const auto& observation = frame_observation.observation;
    const auto& image = frame_observation.image;
    sm::kinematics::Transformation T_t_c;
    
    // Now estimateTransformation uses initialized, mathematically safe intrinsics!
    bool pose_success = camera_calibrator->camera_geometry()->estimateTransformation(observation, T_t_c);
    
    cv::Mat display_img = image.empty() ? cv::Mat::zeros(img_rows, img_cols, CV_8UC3) : image.clone();
    if (display_img.channels() == 1) {
      cv::cvtColor(display_img, display_img, cv::COLOR_GRAY2BGR);
    }

    kalibr2::ros::qa::QaErrorCode status_code = kalibr2::ros::qa::QaErrorCode::POSE_ESTIMATION_FAILED;
    
    if (pose_success) {
      bool is_valuable = qa_tracker->evaluateFrameQuality(image, T_t_c, observation, status_code);
      
      if (is_valuable) {
        valid_frame_observations.push_back(frame_observation);

        auto target_pose_dv = kalibr2::tools::AddPoseDesignVariable(problem, T_t_c);
        target_pose_dvs.push_back(target_pose_dv);

        auto T_cam_w = target_pose_dv->toExpression().inverse();
        camera_calibrator->AddReprojectionErrorsForView(problem, observation, T_cam_w, target, invR);
      }
    } else {
      qa_tracker->RecordOutcome(status_code);
    }

    // GUI Visuals
    std::vector<cv::Point2f> corners;
    observation.getCornersImageFrame(corners);
    for (const auto& c : corners) {
      cv::circle(display_img, c, 3, cv::Scalar(0, 255, 0), -1);
    }
    cv::Scalar text_color = (status_code == kalibr2::ros::qa::QaErrorCode::ACCEPTED) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::putText(display_img, qaErrorCodeToString(status_code), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);
    
    cv::imshow(window_name, display_img);
    cv::imshow("Heatmap - " + camera_name, qa_tracker->getHeatmapImage());
    qa_tracker->spinROSOnce();
    cv::waitKey(1);
  }
  
  std::cout << "[" << camera_name << "] QA Filtered from " << frame_observations.size() << " down to "
            << valid_frame_observations.size() << " valid poses." << std::endl;

  if (target_pose_dvs.empty()) {
    SM_WARN("QA rejected all observations for camera %s", camera_name.c_str());
    cv::destroyWindow(window_name);
    cv::destroyWindow("Heatmap - " + camera_name);
    return false;
  }
  
  // Replace the original array with strictly QA vetted observations.
  frame_observations = valid_frame_observations;

  cv::destroyWindow(window_name);
  cv::destroyWindow("Heatmap - " + camera_name);

  // Proceed with Kalibr's original optimization logic on the safe valid_observations
  auto optimizer = kalibr2::tools::CreateDefaultOptimizer();
  optimizer.setProblem(problem);
  auto retval = optimizer.optimize();
  return !retval.linearSolverFailure;
}

int main(int argc, char** argv) {
  CLI::App app{"kalibr_calibrate_cameras - Calibrate multiple cameras from ROS bag data"};
  argv = app.ensure_utf8(argv);

  std::string bag_path;
  app.add_option("-c,--config", bag_path, "Full path to calibration configuration YAML file.")
      ->required()
      ->check(CLI::ExistingFile);
  std::string topic_name;
  app.add_option("-t,--topic", topic_name,"Topic name for Camera Data");

  std::string output_dir;
  app.add_option("-o,--output-dir", output_dir, "Directory to save the calibration results.")
      ->required()
      ->check(CLI::ExistingDirectory);

  double approx_sync_tolerance(0.02);
  app.add_option("--approx-sync-tolerance", approx_sync_tolerance,
                 "Tolerance for approximate synchronization of observations across cameras (in seconds).");

  double mutual_information_tolerance = 0.2;
  app.add_option("--mi-tol", mutual_information_tolerance,
                 "The tolerance on the mutual information for adding an image in the incremental calibration. Higher "
                 "means fewer images will be added. Use -1 to force all images.");

  std::optional<size_t> max_batches;
  app.add_option("--max-batches", max_batches,
                 "Maximum number of batches to accept during incremental calibration. If not specified, all batches "
                 "will be processed.");

  std::optional<size_t> max_observations;
  app.add_option("--max-observations", max_observations,
                 "Soft target number of detections to report per camera. Extraction continues through all frames for QA "
                 "coverage.");

  size_t frame_stride = 4;
  app.add_option("--frame-stride", frame_stride,
                 "Process only every Nth frame during observation extraction (default: 4).")
      ->check(CLI::Range(static_cast<size_t>(1), std::numeric_limits<size_t>::max()));

  bool verbose = false;
  app.add_flag("--verbose", verbose, "Enable verbose output during calibration.");

  CLI11_PARSE(app, argc, argv);
  
  // ROS Node Init 
  rclcpp::init(argc, argv);

  // | --- User side setup --- |
  // "/kalibr/aslam_offline_calibration/kalibr2_ros/calibration_config.yaml"
  CalibrationConfig config = kalibr2::ros::ConfigFromYaml(bag_path, topic_name);

  std::vector<boost::shared_ptr<kalibr2::CameraCalibratorBase>> camera_calibrators;
  for (const auto& camera_config : config.cameras) {
    auto calibrator = kalibr2::CreateCalibrator(camera_config.model);
    camera_calibrators.push_back(calibrator);
  }

  // |---- Extract observations for each camera ----|
  std::vector<std::vector<FrameObservation>> frame_observations_by_camera(camera_calibrators.size());

  // Use multithreading to fill observations_by_camera in parallel.
  std::vector<std::thread> threads;
  threads.reserve(camera_calibrators.size());

  for (size_t camera_id = 0; camera_id < camera_calibrators.size(); ++camera_id) {
    // Capture camera_id by value to avoid iterator-capture issues.
    const size_t id = camera_id;
    threads.emplace_back([&config, &camera_calibrators, &frame_observations_by_camera, id, max_observations,
                          frame_stride]() {
      const auto& camera_config = config.cameras.at(id);
      auto detector = aslam::cameras::GridDetector(camera_calibrators.at(id)->camera_geometry(), config.target);
      frame_observations_by_camera.at(id) = get_observations_from_camera(
            *camera_config.reader, detector, max_observations, camera_config.camera_name, frame_stride);

    });
  }

  for (auto& t : threads) {
    t.join();
  }

  for (size_t i = 0; i < frame_observations_by_camera.size(); ++i) {
    std::cout << config.cameras[i].camera_name << " collected " << frame_observations_by_camera[i].size()
              << " observation-image pairs." << std::endl;
  }

  // |---- Get initial intrinsic guess for each camera ----|
  for (size_t camera_id = 0; camera_id < camera_calibrators.size(); ++camera_id) {
    const auto& camera_config = config.cameras.at(camera_id);
    bool success =
        CalibrateSingleCameraQA(frame_observations_by_camera.at(camera_id), camera_calibrators.at(camera_id),
                                config.target, camera_config.focal_length, camera_config.camera_name);
    if (!success) {
      throw std::runtime_error("Failed to calibrate intrinsics from observations for camera ID: " +
                               std::to_string(camera_id));
    }
  }

  std::vector<std::vector<aslam::cameras::GridCalibrationTargetObservation>> observations_by_camera;
  observations_by_camera.resize(camera_calibrators.size());
  for (size_t camera_id = 0; camera_id < camera_calibrators.size(); ++camera_id) {
    observations_by_camera[camera_id].reserve(frame_observations_by_camera[camera_id].size());
    for (const auto& frame_observation : frame_observations_by_camera[camera_id]) {
      observations_by_camera[camera_id].push_back(frame_observation.observation);
    }
    std::cout << config.cameras[camera_id].camera_name << " retained " << observations_by_camera[camera_id].size()
              << " QA-filtered observations for multi-camera calibration." << std::endl;
  }

  // | ---- Sync observations across cameras ----|
  auto synced_sets = std::vector<kalibr2::SyncedSet>();
  for (const auto& sync_set :
       kalibr2::SynchronizedObservationView(observations_by_camera, aslam::Duration(approx_sync_tolerance))) {
    synced_sets.push_back(sync_set);
    for (size_t i = 0; i < sync_set.size(); ++i) {
      if (sync_set[i].has_value()) {
      } else {
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
      std::cout << "Best pair for camera " << config.cameras[i].camera_name << ": "
                << config.cameras[best_camera_pair].camera_name << std::endl;
      std::cout << "Distance to camera " << config.cameras[i].camera_name << ": " << result.GetNodeDistance(i)
                << std::endl;
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
    if (tf_it == optimal_baselines.end()) {
      throw std::runtime_error("No transform found for camera pair " + std::to_string(i) + " and " +
                               std::to_string(i + 1));
    }
    baseline_guesses.push_back(tf_it->second);
  }

  // | ---- Refine guess in full batch optimization ----|
  auto baselines =
      kalibr2::tools::CalibrateMultiCameraRig(camera_calibrators, synced_sets, config.target, baseline_guesses);

  // |||| (END) THIS IS WHERE THE original call calls getInitialGuesses to get the baselines ||||

  // | ---- Initialize shared design variables (created once, reused across all batches) ----|
  // 1. Create baseline design variables (SHARED across all batches)
  std::vector<kalibr2::tools::PoseDesignVariables> baseline_pose_dvs;
  for (const auto& baseline : baselines) {
    // Create design variables for the rotation and translation
    auto q_Dv = boost::make_shared<aslam::backend::RotationQuaternion>(baseline.q());
    q_Dv->setActive(true);

    auto t_Dv = boost::make_shared<aslam::backend::EuclideanPoint>(baseline.t());
    t_Dv->setActive(true);

    auto transformation =
        boost::make_shared<aslam::backend::TransformationBasic>(q_Dv->toExpression(), t_Dv->toExpression());

    baseline_pose_dvs.push_back({q_Dv, t_Dv, transformation});
  }

  // 2. Create landmark design variables (SHARED across all batches)
  std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>> landmark_dvs;
  for (size_t i = 0; i < config.target->size(); ++i) {
    landmark_dvs.push_back(
        boost::make_shared<aslam::backend::HomogeneousPoint>(sm::kinematics::toHomogeneous(config.target->point(i))));
  }

  // | ---- Setup incremental estimator ----|
  auto ic_options = aslam::calibration::IncrementalEstimator::Options();
  ic_options.infoGainDelta = mutual_information_tolerance;
  ic_options.checkValidity = true;
  ic_options.verbose = verbose;

  auto linear_solver_options = aslam::calibration::LinearSolverOptions();
  linear_solver_options.columnScaling = true;
  linear_solver_options.verbose = verbose;
  linear_solver_options.epsSVD = 1e-6;

  auto optimizer_options = aslam::backend::Optimizer2Options();
  // original code maxes the number based on available cores
  optimizer_options.maxIterations = 20;
  optimizer_options.nThreads = 16;
  optimizer_options.verbose = verbose;

  // This must match the CALIBRATION_GROUP_ID used in CalibrationTools.hpp
  // TODO(frneer): make this code live in a single location to avoid mismatches
  constexpr int CALIBRATION_GROUP_ID = 0;

  auto estimator = aslam::calibration::IncrementalEstimator(CALIBRATION_GROUP_ID, ic_options, linear_solver_options,
                                                            optimizer_options);

  // TODO(frneer): randomize the order of sync sets on request
  // std::random_shuffle(synced_sets.begin(), synced_sets.end());
  std::vector<kalibr2::tools::BatchProblemStruct> batch_problems;
  size_t accepted_batches = 0;
  size_t processed_batches = 0;
  for (auto& synced_set : synced_sets) {
    if (max_batches.has_value() && accepted_batches >= max_batches.value()) {
      std::cout << "Reached maximum number of accepted batches (" << max_batches.value() << "). Stopping." << std::endl;
      break;
    }

    auto T_tc_guess = kalibr2::tools::getTargetPoseGuess(camera_calibrators, synced_set, baselines);
    auto batch_problem_struct =
        kalibr2::tools::CreateBatchProblem(camera_calibrators, synced_set, T_tc_guess, baseline_pose_dvs, landmark_dvs);
    auto batch_return_value = estimator.addBatch(batch_problem_struct.problem);
    processed_batches++;

    if (batch_return_value.numIterations > optimizer_options.maxIterations) {
      throw std::runtime_error("Optimizer reached max iterations. Something went wrong.");
    }

    batch_problems.push_back(batch_problem_struct);
    if (batch_return_value.batchAccepted) {
      accepted_batches++;
    } else {
      std::cout << "Batch rejected." << std::endl;
    }
  }
  std::cout << "Accepted " << accepted_batches << " batches (processed " << processed_batches << " out of "
            << synced_sets.size() << " total)." << std::endl;

  std::cout << "\n--- Final Camera Parameters ---" << std::endl;
  for (size_t i = 0; i < camera_calibrators.size(); ++i) {
    auto& camera_calibrator = camera_calibrators[i];
    camera_calibrator->PrintReprojectionErrorStatistics();

    // Export to CameraInfo YAML
    const auto& camera_config = config.cameras.at(i);
    auto [width, height] = camera_config.reader->GetImageSize();
    std::string output_filename = "calibration_" + camera_config.camera_name + ".yaml";
    std::string output_filepath = output_dir + "/" + output_filename;
    kalibr2::ros::CalibratorToYAML(camera_calibrator, camera_config.model, camera_config.camera_name, width, height,
                                   output_filepath);
    std::cout << "Exported calibration to: " << output_filepath << std::endl;
  }

  // Export all extrinsics (use the optimized baselines from the DVs)
  std::vector<sm::kinematics::Transformation> optimized_baselines;
  for (const auto& pose_dv : baseline_pose_dvs) {
    // Get the transformation matrix from the expression and convert to sm::kinematics::Transformation
    optimized_baselines.push_back(
        sm::kinematics::Transformation(pose_dv.transformation->toExpression().toTransformationMatrix()));
  }

  if (optimized_baselines.size() == 1) {
    // Single transform - export as TransformStamped
    std::string transform_filename =
        "transform_" + config.cameras[0].camera_name + "_to_" + config.cameras[1].camera_name + ".yaml";
    std::string transform_filepath = output_dir + "/" + transform_filename;
    auto tf_msg = kalibr2::ros::TransformationToROS(optimized_baselines[0],
                                                    {config.cameras[0].camera_name, config.cameras[1].camera_name});
    kalibr2::ros::transformStampedToYAML(tf_msg, transform_filepath);
    std::cout << "Exported transform to: " << transform_filepath << std::endl;
  } else {
    // Multiple transforms - export as TFMessage
    std::vector<std::pair<std::string, std::string>> frames;
    for (size_t i = 0; i < optimized_baselines.size(); ++i) {
      frames.emplace_back(config.cameras[i].camera_name, config.cameras[i + 1].camera_name);
    }

    auto tf_message = kalibr2::ros::TransformationsToTFMessage(optimized_baselines, frames);
    std::string tf_message_filepath = output_dir + "/camera_chain_transforms.yaml";
    kalibr2::ros::tfMessageToYAML(tf_message, tf_message_filepath);
    std::cout << "Exported all transforms to: " << tf_message_filepath << std::endl;
  }
  
  // Shutdown QA node scope
  rclcpp::shutdown();
}
