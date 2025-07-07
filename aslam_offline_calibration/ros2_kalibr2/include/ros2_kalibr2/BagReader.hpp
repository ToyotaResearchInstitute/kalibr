#pragma once

#include <filesystem>
#include <string>
#include <memory>

#include <opencv2/core.hpp>

#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <aslam/Time.hpp>
#include <kalibr2/Image.hpp>



template<typename MessageT>
Image image_from_message(const MessageT& msg) {
  Image img;
  img.timestamp = aslam::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);
  img.image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

  return img;
}

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

template<typename MessageT>
class BagImageReader : public ImageReader {
  public:
    BagImageReader(std::unique_ptr<rosbag2_cpp::Reader> reader) : reader_(std::move(reader)) {}

    Image ReadNext() override {
      // if (!reader_->has_next()) {
      //   throw std::runtime_error("No more messages in the bag.");
      // }
      auto msg = reader_->read_next<MessageT>();
      return image_from_message(msg);
    }

    bool HasNext() const override {
      return reader_->has_next();
    }
  private:
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};



std::unique_ptr<ImageReader> create_bag_reader(const std::string& bag_file_path, const std::string& topic) {
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
        return std::make_unique<BagImageReader<sensor_msgs::msg::Image>>(std::move(reader));
      } else if (topic_metadata.type == "sensor_msgs/msg/CompressedImage") {
        return std::make_unique<BagImageReader<sensor_msgs::msg::CompressedImage>>(std::move(reader));
      } else {
        throw std::runtime_error("Unsupported image type: " + topic_metadata.type);
      // }
      }
    }
  }
}




// class BagReader {
// public:
//   BagReader(const std::string &bag_file_path) {
//     auto bag_metadata = get_bag_metadata(bag_file_path);
//     auto storage_options = get_storage_options(bag_file_path, bag_metadata);
//     rosbag2_cpp::ConverterOptions converter_options;
//     converter_options.input_serialization_format = "cdr"; // Assuming the serialization format is CDR
//     converter_options.output_serialization_format = "cdr"; // Assuming the output format is also CDR
//     reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
//     reader_->open(storage_options, converter_options);
//     auto topics_metadata = reader_->get_all_topics_and_types();
//     topics_metadata_map_ = std::map<std::string, rosbag2_storage::TopicMetadata>();
//     for (const auto& topic_metadata : topics_metadata) {
//       topics_metadata_map_[topic_metadata.name] = topic_metadata;
//     }
//   }

//   rosbag2_storage::TopicMetadata GetTopicMetadata(const std::string& topic) const {
//     auto it = topics_metadata_map_.find(topic);
//     if (it == topics_metadata_map_.end()) {
//       throw std::runtime_error("Topic not found: " + topic);
//     }
//     return it->second;
//   }

//   void FastForward(const rcutils_time_point_value_t & time) {
//     reader_->seek(time);
//   }

//   bool HasNext() const {
//     return reader_->has_next();
//   }

//   Image ReadImageMessage(const std::string& topic) {
//     auto type = GetTopicMetadata(topic).type;

//     rosbag2_storage::StorageFilter filter;
//     filter.topics = {topic};
//     reader_->set_filter(filter);

//     if (type == "sensor_msgs/msg/Image") {
//       auto img_msg = reader_->read_next<sensor_msgs::msg::Image>();
//     else if (type == "sensor_msgs/msg/CompressedImage") {
//       auto img_msg = reader_->read_next<sensor_msgs::msg::CompressedImage>();
//     } else {
//       throw std::runtime_error("Unsupported image type: " + type);
//     }

//     return image_from_message(img_msg);
//   }

// private:
//   std::unique_ptr<rosbag2_cpp::Reader> reader_;
//   std::map<std::string, rosbag2_storage::TopicMetadata> topics_metadata_map_;
// };
