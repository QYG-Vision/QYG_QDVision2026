#ifndef RM_AUTO_RECORD__RECORD_NODE_HPP_
#define RM_AUTO_RECORD__RECORD_NODE_HPP_

#include <rmw/qos_profiles.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <queue>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <vector>

namespace rm_auto_record
{
class RecordNode : public rclcpp::Node
{
public:
  explicit RecordNode(const rclcpp::NodeOptions & options);
  ~RecordNode();

private:
  void startRecording();
  void stopRecording();
  void writingThread();
  void gimbalCmdCallback(const rm_interfaces::msg::GimbalCmd::ConstSharedPtr msg);
  void checkStopCondition();

  std::shared_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
  std::string uri_;

  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> generic_subscriptions_;
  std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> tf_message_subscriptions_;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_message_serialization_;

  enum RecordState {
    BUFFERING,
    RECORDING,
  };
  std::atomic<int> record_state_;

  struct TopicInfo
  {
    std::string topic_name;
    std::string topic_type;
  };

  std::vector<TopicInfo> topic_info_;

  // Write queue & writer thread
  std::thread writer_thread_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::atomic<bool> stop_thread_;

  struct MessageQueueItem
  {
    std::shared_ptr<rclcpp::SerializedMessage> msg;
    std::string topic_name;
    std::string topic_type;
    rclcpp::Time timestamp;
  };

  std::queue<MessageQueueItem> message_queue_;

  // Protects writer_ access across callback thread and writer thread
  std::mutex writer_mutex_;

  // Circular buffer for pre-trigger data
  std::deque<MessageQueueItem> circular_buffer_;
  std::mutex buffer_mutex_;

  double buffer_duration_;
  double stop_delay_;

  // GimbalCmd monitoring
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
  std::atomic<int64_t> last_positive_distance_ns_;
  std::atomic<bool> close_requested_;
  rclcpp::TimerBase::SharedPtr stop_check_timer_;

  // Protects state transitions (startRecording / stopRecording)
  std::mutex state_mutex_;
};

}  // namespace rm_auto_record

#endif  // RM_AUTO_RECORD__RECORD_NODE_HPP_
