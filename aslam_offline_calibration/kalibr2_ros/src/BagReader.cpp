#include "kalibr2_ros/BagReader.hpp"

#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

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
    auto img = image_from_message(msg);
    if (image_width_ == 0 || image_height_ == 0) {
      image_width_ = img.image.cols;
      image_height_ = img.image.rows;
    }
    return img;
  }

  bool HasNext() const override { return reader_->has_next(); }

  size_t MessageCount() override { return image_count_; }

 private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  size_t image_count_;
};

template <typename MessageT>
class TopicImageReader : public ImageReader {
 public:
  TopicImageReader(const std::string& topic) {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("topic_image_reader_node_" + std::to_string(std::rand()));
    
    sub_ = node_->create_subscription<MessageT>(
        topic, rclcpp::SensorDataQoS(),
        [this](const std::shared_ptr<const MessageT> msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          queue_.push(*msg);
          cv_.notify_one();
        });
        
    spin_thread_ = std::thread([this]() {
      rclcpp::spin(node_);
    });
  }

  ~TopicImageReader() override {
    if (node_) {
      rclcpp::shutdown();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  Image ReadNext() override {
    MessageT msg;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() { return !queue_.empty(); });
      msg = queue_.front();
      queue_.pop();
    }
    auto img = image_from_message(msg);
    if (image_width_ == 0 || image_height_ == 0) {
      image_width_ = img.image.cols;
      image_height_ = img.image.rows;
    }
    return img;
  }

  bool HasNext() const override {
    if (!rclcpp::ok() || !is_active_) {
      return false;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if (!queue_.empty()) {
      return true;
    }
    // Timeout approach: assume stream ended if no frame arrives in 3 seconds
    if (cv_.wait_for(lock, std::chrono::seconds(3), [this]() { return !queue_.empty(); })) {
      return true;
    }
    
    is_active_ = false;
    // Log the end of stream so the user knows it finished
    RCLCPP_INFO(node_->get_logger(), "No images received for 3 seconds. Ending live stream capture.");
    return false;
  }

  size_t MessageCount() override { 
    // Live topics do not have a known limit.
    return 0; 
  }

 private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Subscription<MessageT>::SharedPtr sub_;
  std::queue<MessageT> queue_;
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  std::thread spin_thread_;
  mutable bool is_active_ = true;
};

}  // namespace

//PIPELINE:: IMAGE READERS ARE HERE. LIVE IMAGE READER IS NEEDED

std::unique_ptr<ImageReader> TopicImageReaderFactory::create(const std::string& topic) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  auto node = rclcpp::Node::make_shared("topic_discovery_node_" + std::to_string(std::rand()));
  std::string topic_type = "";
  
  // Discover topic type by polling the ROS graph. Wait up to 1 second.
  for (int i = 0; i < 10; ++i) {
    auto topics_and_types = node->get_topic_names_and_types();
    if (topics_and_types.find(topic) != topics_and_types.end()) {
      const auto& types = topics_and_types.at(topic);
      if (!types.empty()) {
        topic_type = types.front();
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (topic_type == "sensor_msgs/msg/CompressedImage") {
    return std::make_unique<TopicImageReader<sensor_msgs::msg::CompressedImage>>(topic);
  } else if (topic_type == "sensor_msgs/msg/Image" || topic_type.empty()) {
    if (topic_type.empty()) {
      RCLCPP_WARN(node->get_logger(), "Could not dynamically discover type for topic '%s'. Falling back to 'sensor_msgs/msg/Image'", topic.c_str());
    }
    return std::make_unique<TopicImageReader<sensor_msgs::msg::Image>>(topic);
  } else {
    throw std::runtime_error("Unsupported live topic image type: " + topic_type);
  }
}

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
