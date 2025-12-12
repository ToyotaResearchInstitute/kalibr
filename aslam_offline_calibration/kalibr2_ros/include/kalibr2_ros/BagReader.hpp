#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include <aslam/Time.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <kalibr2/Image.hpp>
#include <opencv2/core.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace kalibr2 {

namespace ros {

class BagImageReaderFactory {
 public:
  static std::unique_ptr<ImageReader> create(const std::string& bag_file_path, const std::string& topic);
};

}  // namespace ros

}  // namespace kalibr2
