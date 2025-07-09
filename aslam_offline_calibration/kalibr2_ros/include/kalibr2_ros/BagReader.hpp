#pragma once

#include <filesystem>
#include <string>
#include <memory>

#include <opencv2/core.hpp>

#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <aslam/Time.hpp>
#include <kalibr2/Image.hpp>

namespace kalibr2 {

namespace ros {

template<typename MessageT>
Image image_from_message(const MessageT& msg) {
  kalibr2::Image img;
  img.timestamp = aslam::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);
  img.image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

  return img;
}


template<typename MessageT>
class BagImageReader : public kalibr2::ImageReader {
  public:
  BagImageReader(std::unique_ptr<rosbag2_cpp::Reader> reader) : reader_(std::move(reader)) {}

  kalibr2::Image ReadNext() override {
    auto msg = reader_->read_next<MessageT>();
    return image_from_message(msg);
  }

  bool HasNext() const override {
    return reader_->has_next();
  }
  private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};


rosbag2_storage::BagMetadata get_bag_metadata(const std::string& bag_file_path);


rosbag2_storage::StorageOptions get_storage_options(const std::string& bag_file_path, const rosbag2_storage::BagMetadata& bag_metadata);


std::unique_ptr<ImageReader> create_bag_reader(const std::string& bag_file_path, const std::string& topic);

} // namespace ros

} // namespace kalibr2
