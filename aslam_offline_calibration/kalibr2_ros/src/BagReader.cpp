#include "kalibr2_ros/BagReader.hpp"


namespace kalibr2 {

namespace ros {

rosbag2_storage::BagMetadata get_bag_metadata(const std::string& bag_file_path) {
  auto metadata_directory = std::filesystem::path(bag_file_path).parent_path();
  auto metadata_io = rosbag2_storage::MetadataIo();
  return metadata_io.read_metadata(metadata_directory.string());
}


rosbag2_storage::StorageOptions get_storage_options(const std::string& bag_file_path, const rosbag2_storage::BagMetadata& bag_metadata) {
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_file_path;
  storage_options.storage_id = bag_metadata.storage_identifier;
  return storage_options;
}


std::unique_ptr<kalibr2::ImageReader> create_bag_reader(const std::string& bag_file_path, const std::string& topic) {
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

  for (const auto& topic_metadata : reader->get_all_topics_and_types()) {
    if (topic_metadata.name == topic) {
      if (topic_metadata.type == "sensor_msgs/msg/Image") {
        return std::make_unique<kalibr2::ros::BagImageReader<sensor_msgs::msg::Image>>(std::move(reader));
      } else if (topic_metadata.type == "sensor_msgs/msg/CompressedImage") {
        return std::make_unique<kalibr2::ros::BagImageReader<sensor_msgs::msg::CompressedImage>>(std::move(reader));
      } else {
        throw std::runtime_error("Unsupported image type: " + topic_metadata.type);
      }
    }
  }
}

} // namespace ros

} // namespace kalibr2
