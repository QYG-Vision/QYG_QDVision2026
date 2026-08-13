// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#include "rm_auto_record/record_node.hpp"

#include <rclcpp/subscription_options.hpp>

namespace rm_auto_record
{
namespace
{
constexpr const char kTfMessageType[] = "tf2_msgs/msg/TFMessage";
}  // namespace

RecordNode::RecordNode(const rclcpp::NodeOptions & options) : Node("rm_auto_record", options)
{
  record_state_ = BUFFERING;
  stop_thread_ = false;
  last_positive_distance_ns_.store(this->now().nanoseconds());
  close_requested_.store(false);

  uri_ = this->declare_parameter<std::string>("uri", "/ros_ws/");
  buffer_duration_ = this->declare_parameter<double>("buffer_duration", 10.0);
  stop_delay_ = this->declare_parameter<double>("stop_delay", 3.0);

  RCLCPP_INFO(get_logger(), "Record at: %s", uri_.c_str());
  RCLCPP_INFO(
    get_logger(), "Buffer duration: %.1f s, Stop delay: %.1f s", buffer_duration_, stop_delay_);

  auto topic_name =
    this->declare_parameter<std::vector<std::string>>("topic_name", std::vector<std::string>());
  auto topic_type =
    this->declare_parameter<std::vector<std::string>>("topic_type", std::vector<std::string>());

  if (topic_name.size() != topic_type.size()) {
    RCLCPP_ERROR(get_logger(), "Topic name and type size not match.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Record topics number: %ld", topic_name.size());
  for (size_t i = 0; i < topic_name.size(); i++) {
    TopicInfo topic_info;
    topic_info.topic_name = topic_name[i];
    topic_info.topic_type = topic_type[i];
    RCLCPP_INFO(
      get_logger(), "Record topic: %s, type: %s", topic_info.topic_name.c_str(),
      topic_info.topic_type.c_str());
    topic_info_.push_back(topic_info);
  }

  RCLCPP_INFO(get_logger(), "Record node initialized, waiting for trigger (distance > 0).");

  writer_thread_ = std::thread(&RecordNode::writingThread, this);

  gimbal_cmd_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "/armor_solver/cmd_gimbal", rclcpp::SensorDataQoS(),
    std::bind(&RecordNode::gimbalCmdCallback, this, std::placeholders::_1));

  stop_check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&RecordNode::checkStopCondition, this));

  auto enqueue_serialized = [this](
                              std::shared_ptr<rclcpp::SerializedMessage> msg,
                              const std::string & t_name, const std::string & t_type) {
    MessageQueueItem item;
    item.msg = std::move(msg);
    item.topic_name = t_name;
    item.topic_type = t_type;
    item.timestamp = this->now();

    if (record_state_.load() == RECORDING) {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      message_queue_.push(std::move(item));
      queue_cv_.notify_one();
    } else {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      auto cutoff = item.timestamp;
      circular_buffer_.push_back(std::move(item));
      while (!circular_buffer_.empty() &&
             (cutoff - circular_buffer_.front().timestamp).seconds() > buffer_duration_) {
        circular_buffer_.pop_front();
      }
    }
  };

  for (const auto & info : topic_info_) {
    const std::string & t_name = info.topic_name;
    const std::string & t_type = info.topic_type;

    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    if (t_name.find("tf_static") != std::string::npos) {
      qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    }

    try {
      if (t_type == kTfMessageType) {
        rclcpp::SubscriptionOptions sub_opts;
        if (t_name.find("tf_static") != std::string::npos) {
          sub_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        }
        auto sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
          t_name, qos,
          [this, t_name, t_type, enqueue_serialized](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
            auto serialized = std::make_shared<rclcpp::SerializedMessage>();
            tf_message_serialization_.serialize_message(msg.get(), serialized.get());
            enqueue_serialized(std::move(serialized), t_name, t_type);
          },
          sub_opts);
        tf_message_subscriptions_.push_back(std::move(sub));
      } else {
        auto sub = this->create_generic_subscription(
          t_name, t_type, qos,
          [this, t_name, t_type,
           enqueue_serialized](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            enqueue_serialized(std::move(msg), t_name, t_type);
          });
        generic_subscriptions_.push_back(std::move(sub));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to subscribe to %s: %s", t_name.c_str(), e.what());
    }
  }
}

RecordNode::~RecordNode()
{
  stop_thread_ = true;
  queue_cv_.notify_one();
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }

  std::lock_guard<std::mutex> wlock(writer_mutex_);
  if (writer_) {
    writer_->close();
  }
}

void RecordNode::gimbalCmdCallback(const rm_interfaces::msg::GimbalCmd::ConstSharedPtr msg)
{
  if (msg->distance > 0) {
    last_positive_distance_ns_.store(this->now().nanoseconds());

    if (record_state_.load() == BUFFERING) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (record_state_.load() == BUFFERING) {
        startRecording();
      }
    }
  }
}

void RecordNode::checkStopCondition()
{
  if (record_state_.load() != RECORDING) return;

  auto now_ns = this->now().nanoseconds();
  double elapsed = (now_ns - last_positive_distance_ns_.load()) / 1e9;

  if (elapsed > stop_delay_) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (record_state_.load() != RECORDING) return;

    RCLCPP_INFO(
      get_logger(), "Distance <= 0 for %.1f s (threshold %.1f s), stopping.", elapsed, stop_delay_);
    stopRecording();
  }
}

void RecordNode::startRecording()
{
  if (close_requested_.exchange(false)) {
    std::lock_guard<std::mutex> wlock(writer_mutex_);
    if (writer_) {
      writer_->close();
      writer_.reset();
    }
  }

  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  auto yyyy_mm_dd_hh_mm_ss = std::localtime(&now_c);
  std::string bag_name = "record_" + std::to_string(yyyy_mm_dd_hh_mm_ss->tm_year + 1900) + "年" +
                         std::to_string(yyyy_mm_dd_hh_mm_ss->tm_mon + 1) + "月" +
                         std::to_string(yyyy_mm_dd_hh_mm_ss->tm_mday) + "日" +
                         std::to_string(yyyy_mm_dd_hh_mm_ss->tm_hour) + "点" +
                         std::to_string(yyyy_mm_dd_hh_mm_ss->tm_min) + "分" +
                         std::to_string(yyyy_mm_dd_hh_mm_ss->tm_sec);

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = uri_ + bag_name;
  storage_options.storage_id = "mcap";
  storage_options.max_bagfile_duration = 30;

  const rosbag2_cpp::ConverterOptions converter_options(
    {rmw_get_serialization_format(), rmw_get_serialization_format()});

  {
    std::lock_guard<std::mutex> wlock(writer_mutex_);
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
    writer_->open(storage_options, converter_options);

    for (const auto & topic_info : topic_info_) {
      writer_->create_topic(
        {topic_info.topic_name, topic_info.topic_type, rmw_get_serialization_format(), ""});
    }
  }

  RCLCPP_INFO(get_logger(), "Start recording, saving to: %s", storage_options.uri.c_str());

  {
    std::lock_guard<std::mutex> block(buffer_mutex_);
    std::lock_guard<std::mutex> qlock(queue_mutex_);
    while (!circular_buffer_.empty()) {
      message_queue_.push(std::move(circular_buffer_.front()));
      circular_buffer_.pop_front();
    }
  }
  queue_cv_.notify_one();

  size_t flushed = message_queue_.size();
  RCLCPP_INFO(get_logger(), "Flushed %zu buffered messages to recording.", flushed);

  record_state_.store(RECORDING);
}

void RecordNode::stopRecording()
{
  record_state_.store(BUFFERING);
  close_requested_.store(true);
  queue_cv_.notify_one();
}

void RecordNode::writingThread()
{
  while (rclcpp::ok() && !stop_thread_) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait(lock, [this]() {
      return !message_queue_.empty() || stop_thread_ || close_requested_.load();
    });

    while (!message_queue_.empty()) {
      auto item = std::move(message_queue_.front());
      message_queue_.pop();
      lock.unlock();

      {
        std::lock_guard<std::mutex> wlock(writer_mutex_);
        if (writer_) {
          auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

          bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            new rcutils_uint8_array_t, [](rcutils_uint8_array_t * msg) {
              auto fini_return = rcutils_uint8_array_fini(msg);
              delete msg;
              if (fini_return != RCUTILS_RET_OK) {
              }
            });

          *bag_message->serialized_data = item.msg->release_rcl_serialized_message();
          bag_message->topic_name = item.topic_name;
          bag_message->time_stamp = item.timestamp.nanoseconds();

          writer_->write(bag_message);
        }
      }

      lock.lock();
    }

    if (close_requested_.load()) {
      lock.unlock();
      {
        std::lock_guard<std::mutex> wlock(writer_mutex_);
        if (writer_) {
          writer_->close();
          writer_.reset();
        }
      }
      close_requested_.store(false);
      RCLCPP_INFO(get_logger(), "Stopped recording, back to buffering.");
    }
  }
}

}  // namespace rm_auto_record

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_record::RecordNode)
