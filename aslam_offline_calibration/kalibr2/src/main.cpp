struct Image {
  acv::Time timestamp;
  cv::Mat data;
}

class BagFile {
  public:
    BagFile(const std::string& path);
    Image extractImage(const std::string& topic);
    std::vector<Image> extractImages(const std::string& topic);
  private:
    std::string path_;
}

typedef Observation = std::optional<GridCalibrationTargetObservation>;

Observation ToObservation(const Image& image, const GridDetector& detector) {
  // the target accesor doesnt exist, why it needs the target if it's built inside the detector?
  // Maybe refactor and build the observation in findTargetNoTransformation...
  auto observation = GridCalibrationTargetObservation(detector.target());
  bool success = findTargetNoTransformation(timestamp, image, observation);
  // delete image copy (save memory)
  observation.clearImage();
  if (success) {
    return observation;
  } else {
    return std::nullopt;
  }
}

// Maybe use std::optional::transform?
std::vector<Observation> ToObservation(std::vector<Image> images, const GridDetector& detector) {
  return std::transform(images.begin(), images.end(), std::back_inserter(std::vector<GridCalibrationTargetObservation>()),
    [&detector](const Image& image) {
      return ToObservation(image, detector);
    });
}

template<typename Model>
class Camera {
  public:
    Camera(size_t id) : id_(id) {
      id_ = id;
      geometry_ = Model::Geometry(id);
      detector_ = GridDetector(geometry_, target_);
    }
    Camera(size_t id, const std::vector<Image>& images) : Camera(id) {
      observations_ = ToObservation(images, detector_);
      bool success = geometry_.initializeIntrinsics(observations_);
      // Do an initial calibration of the intrinsics
      // Don't love to do this in the constructor.
      bool success = initialIntrinsicCalibration();
    }

    // This one needs the observations; geometry; target;
    // Some abstraction of design variables?
    // Maybe this should be a method of the camera model?
    initialIntrinsicCalibration();
  private:
    size_t id_;
    Model::Geometry geometry_; // No dependencies, has an ID for some reason. The type is actually defined in the camera model, lol.
    GridDetector detector_; // Depends on geometry; just for findTarget
    std::vector<Observation> observations_;
}

int main() {
  auto bag_file_path = "path/to/your/bagfile.mcap";
  BagFile bag_file(bag_file_path);

  topics = ["/camera/one_camera/image", "/camera/another_camera/image"];
  models = [DistortedPinholeModel, DistortedPinholeModel];

  // Extract images from the bag file for each camera topic
  std::vector<std::vector<Image>> images_per_camera;
  for (const auto& topic : topics) {
    auto camera_images = bag_file.extractImages(topic);
    images_per_camera.push_back(camera_images);
  }


  std::vector<Camera> cameras;
  for (size_t i = 0; i < images_per_camera.size(); ++i) {
    auto camera_images = images_per_camera[i];
    auto model = models[i];
    cameras.push_back(Camera<model>(i, camera_images));
  }

  // Initialize extrinsics
  // --- This part is not yet thought trhough ---
  // This one has a way to retrieve shared observations between cameras
  graph = BuildGraph(cameras);
  baseline_guesses = graph.getInitialGuesses();

  // This one has a way to retrieve shared observations between cameras
  calibrator = CameraCalibrator(cameras, baseline_guesses)
  foreach timestamp {
    // T_TC_Guess is retrieved from the graph as the best camera transform available...
    calibrator.addTargetView(tiemstamp, cam_id, observation, T_tc_guess)
  }

  // Then there's a cleanup of some batches
  calibrator.cleanupBatches();

  // Then save the results somewhere
  return 0;
}
