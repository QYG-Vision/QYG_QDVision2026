// ros2
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// c++ 
#include <chrono>
#include <atomic>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
// Project
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/vision_state_machine.hpp"

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options)
  : Node("hik_camera", options),
    state_machine_(qd::utils::VisionStateMachine::getInstance())
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode with VisionStateMachine!");

    heartbeat_ = qd::HeartBeatPublisher::create(this);

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(rclcpp::KeepLast(10));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);

    state_machine_.initialize(this);
    if (!state_machine_.initializeCamera(qd::utils::CameraType::CAMERA_NEAR_RANGE)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize near-range camera in VisionStateMachine");
      throw std::runtime_error("Failed to initialize camera in VisionStateMachine");
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread(&HikCameraNode::captureLoop, this);
  }

  ~HikCameraNode() override
  {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  /**
   * @brief 从视觉状态机循环取帧并发布 ROS 图像与内参消息。
   * @note 时间戳使用状态机返回的曝光中点时间，失败超过阈值会触发节点退出。
   */
  void captureLoop()
  {
    RCLCPP_INFO(this->get_logger(), "Publishing image from VisionStateMachine!");

    while (rclcpp::ok() && running_) {
      cv::Mat image;
      sensor_msgs::msg::CameraInfo camera_info_msg;
      std::chrono::system_clock::time_point timestamp;

      if (!state_machine_.getCurrentImage(image, camera_info_msg, timestamp)) {
        RCLCPP_WARN(this->get_logger(), "Failed to get frame from VisionStateMachine");
        fail_count_++;
      } else {
        publishFrame(image, camera_info_msg, timestamp);
        fail_count_ = 0;
        heartbeat_->publish();
      }

      if (fail_count_ > 5) {
        RCLCPP_FATAL(this->get_logger(), "Camera failed!");
        rclcpp::shutdown();
      }
    }
  }

  /**
   * @brief 发布当前帧对应的图像与相机内参消息。
   * @param image 当前图像数据。
   * @param camera_info 当前帧对应的相机内参。
   * @param timestamp 当前帧曝光中点时间戳。
   */
  void publishFrame(
    const cv::Mat & image,
    sensor_msgs::msg::CameraInfo camera_info,
    const std::chrono::system_clock::time_point & timestamp)
  {
    const rclcpp::Time ros_time(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count());
    camera_info.header.stamp = ros_time;
    if (camera_info.header.frame_id.empty()) {
      camera_info.header.frame_id = state_machine_.getCurrentTargetFrame();
    }

    cv::Mat publish_image = image.isContinuous() ? image : image.clone();

    auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
    image_msg->header = camera_info.header;
    image_msg->encoding = sensor_msgs::image_encodings::RGB8;
    image_msg->height = static_cast<uint32_t>(publish_image.rows);
    image_msg->width = static_cast<uint32_t>(publish_image.cols);
    image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(
      publish_image.cols * publish_image.elemSize());
    image_msg->data.assign(publish_image.datastart, publish_image.dataend);

    image_pub_->publish(std::move(image_msg));
    camera_info_pub_->publish(std::make_unique<sensor_msgs::msg::CameraInfo>(std::move(camera_info)));
  }

  /**
   * @brief 处理运行时参数更新请求。
   * @param parameters 待更新参数列表。
   * @return 参数更新结果；当前仅支持曝光和增益在线更新。
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        state_machine_.updateExposureTime(param.as_double());
      } else if (param.get_name() == "gain") {
        state_machine_.updateGain(param.as_double());
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
        break;
      }
    }
    return result;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  qd::utils::VisionStateMachine & state_machine_;

  int fail_count_ = 0;
  std::atomic<bool> running_ {true};
  std::thread capture_thread_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  qd::HeartBeatPublisher::SharedPtr heartbeat_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
