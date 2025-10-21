#include "kalibr2_ros/BagReader.hpp"

#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

namespace kalibr2 {

namespace ros {

namespace {

rosbag2_storage::BagMetadata get_bag_metadata(const std::string& bag_file_path) {
  auto metadata_directory = std::filesystem::path(bag_file_path).parent_path();
  auto metadata_io = rosbag2_storage::MetadataIo();
  return metadata_io.read_metadata(metadata_directory.string());
}

rosbag2_storage::StorageOptions get_storage_options(const std::string& bag_file_path,
                                                    const rosbag2_storage::BagMetadata& bag_metadata) {
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_file_path;
  storage_options.storage_id = bag_metadata.storage_identifier;
  return storage_options;
}

std::optional<rosbag2_storage::TopicInformation> get_topic_information(
    const std::string& topic, const std::vector<rosbag2_storage::TopicInformation>& topics_information) {
  const auto it = std::find_if(topics_information.begin(), topics_information.end(),
                               [&topic](const rosbag2_storage::TopicInformation& topic_info) {
                                 return topic_info.topic_metadata.name == topic;
                               });

  return it != topics_information.end() ? std::make_optional(*it) : std::nullopt;
}

/// Transforms a ROS message to an Image.
template <typename MessageT>
Image image_from_message(const MessageT& msg) {
  Image img;
  img.timestamp = aslam::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);
  img.image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image.clone();
  return img;
}

/// Image reader for ROS bag files.
/// This class reads images from a ROS bag file using the specified message
/// type. Uses sequential reading of the images.
template <typename MessageT>
class BagImageReader : public ImageReader {
 public:
  BagImageReader(std::unique_ptr<rosbag2_cpp::Reader> reader, size_t image_count)
      : reader_(std::move(reader)), image_count_(image_count) {}

  Image ReadNext() override {
    auto msg = reader_->read_next<MessageT>();
    return image_from_message(msg);
  }

  bool HasNext() const override { return reader_->has_next(); }

  size_t MessageCount() override { return image_count_; }

 private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  size_t image_count_;
};

}  // namespace

std::unique_ptr<ImageReader> BagImageReaderFactory::create(const std::string& bag_file_path, const std::string& topic) {
  auto bag_metadata = get_bag_metadata(bag_file_path);
  auto storage_options = get_storage_options(bag_file_path, bag_metadata);
  // Assuming the serialization format is CDR
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  reader->open(storage_options, converter_options);

  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic};
  reader->set_filter(filter);

  auto topics_and_types = reader->get_all_topics_and_types();
  const auto it = std::find_if(topics_and_types.begin(), topics_and_types.end(),
                               [&topic](const rosbag2_storage::TopicMetadata& topic_metadata) {
                                 return topic_metadata.name == topic;
                               });

  if (it == topics_and_types.end()) {
    throw std::runtime_error("Topic not found in bag: " + topic);
  }

  auto topic_info = get_topic_information(topic, bag_metadata.topics_with_message_count);
  if (!topic_info.has_value()) {
    throw std::runtime_error("Topic has no messages in bag: " + topic);
  }

  if (it->type == "sensor_msgs/msg/Image") {
    return std::make_unique<BagImageReader<sensor_msgs::msg::Image>>(std::move(reader),
                                                                     topic_info.value().message_count);
  } else if (it->type == "sensor_msgs/msg/CompressedImage") {
    return std::make_unique<BagImageReader<sensor_msgs::msg::CompressedImage>>(std::move(reader),
                                                                               topic_info.value().message_count);
  } else {
    throw std::runtime_error("Unsupported image type: " + it->type);
  }
}

}  // namespace ros

}  // namespace kalibr2
