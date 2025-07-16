#include "kalibr2_ros/BagReader.hpp"

namespace kalibr2 {

namespace ros {

namespace {

rosbag2_storage::BagMetadata get_bag_metadata(
    const std::string& bag_file_path) {
  auto metadata_directory = std::filesystem::path(bag_file_path).parent_path();
  auto metadata_io = rosbag2_storage::MetadataIo();
  return metadata_io.read_metadata(metadata_directory.string());
}

rosbag2_storage::StorageOptions get_storage_options(
    const std::string& bag_file_path,
    const rosbag2_storage::BagMetadata& bag_metadata) {
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_file_path;
  storage_options.storage_id = bag_metadata.storage_identifier;
  return storage_options;
}

/// Transforms a ROS message to an Image.
template <typename MessageT>
Image image_from_message(const MessageT& msg) {
  Image img;
  img.timestamp = aslam::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);
  img.image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)
                  ->image.clone();
  return img;
}

/// Image reader for ROS bag files.
/// This class reads images from a ROS bag file using the specified message
/// type. Uses sequential reading of the images.
template <typename MessageT>
class BagImageReader : public ImageReader {
 public:
  BagImageReader(std::unique_ptr<rosbag2_cpp::Reader> reader)
      : reader_(std::move(reader)) {}

  Image ReadNext() override {
    auto msg = reader_->read_next<MessageT>();
    return image_from_message(msg);
  }

  bool HasNext() const override { return reader_->has_next(); }

 private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

}  // namespace

std::unique_ptr<ImageReader> BagImageReaderFactory::create(const std::string& bag_file_path,
                                               const std::string& topic) {
  auto bag_metadata = get_bag_metadata(bag_file_path);
  auto storage_options = get_storage_options(bag_file_path, bag_metadata);
  // Assuming the serialization format is CDR
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  auto reader =
      rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  reader->open(storage_options, converter_options);

  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic};
  reader->set_filter(filter);

  auto topics_and_types = reader->get_all_topics_and_types();
  const auto it = std::find_if(
      topics_and_types.begin(), topics_and_types.end(),
      [&topic](const rosbag2_storage::TopicMetadata& topic_metadata) {
        return topic_metadata.name == topic;
      });

  if (it == topics_and_types.end()) {
    throw std::runtime_error("Topic not found in bag: " + topic);
  }

  if (it->type == "sensor_msgs/msg/Image") {
    return std::make_unique<BagImageReader<sensor_msgs::msg::Image>>(
        std::move(reader));
  } else if (it->type == "sensor_msgs/msg/CompressedImage") {
    return std::make_unique<BagImageReader<sensor_msgs::msg::CompressedImage>>(
        std::move(reader));
  } else {
    throw std::runtime_error("Unsupported image type: " + it->type);
  }
}

}  // namespace ros

}  // namespace kalibr2
