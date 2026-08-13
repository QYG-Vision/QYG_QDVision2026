#include <atomic>
#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/serial_receive_data.hpp>
#include <rm_interfaces/srv/set_mode.hpp>
#include <rm_utils/heartbeat.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <vector>

struct VideoRecord {
    uint64_t frame_index;
    uint64_t timestamp;
};

struct SerialRecord {
    uint64_t timestamp;
    uint8_t mode;
    float bullet_speed;
    float roll;
    float yaw;
    float pitch;
};

uint64_t parseTimestamp(const std::string& s) {
    size_t pos = s.find('.');
    if (pos != std::string::npos) {
        return std::stoull(s.substr(0, pos));
    }
    return std::stoull(s);
}

std::vector<VideoRecord> readVideoTimestamps(const std::string& filepath) {
    std::vector<VideoRecord> records;
    std::ifstream file(filepath);
    if (!file.is_open())
        return records;

    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty())
            continue;
        std::stringstream ss(line);
        std::string token;
        VideoRecord record;

        if (std::getline(ss, token, ','))
            record.frame_index = std::stoull(token);
        if (std::getline(ss, token, ','))
            record.timestamp = parseTimestamp(token);
        records.push_back(record);
    }
    return records;
}

std::vector<SerialRecord> readSerialData(
    const std::string& filepath,
    const uint64_t min_timestamp,
    const bool filter_by_min
) {
    std::vector<SerialRecord> records;
    std::ifstream file(filepath);
    if (!file.is_open())
        return records;

    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty())
            continue;
        std::stringstream ss(line);
        std::string token;
        SerialRecord record;

        if (std::getline(ss, token, ','))
            record.timestamp = parseTimestamp(token);
        if (std::getline(ss, token, ','))
            record.mode = static_cast<uint8_t>(std::stoi(token));
        if (std::getline(ss, token, ','))
            record.bullet_speed = std::stof(token);
        if (std::getline(ss, token, ','))
            record.roll = std::stof(token);
        if (std::getline(ss, token, ','))
            record.yaw = std::stof(token);
        if (std::getline(ss, token, ','))
            record.pitch = std::stof(token);

        if (filter_by_min && record.timestamp < min_timestamp) {
            continue;
        }
        records.push_back(record);
    }
    return records;
}

namespace qd::auto_replay {

class AutoReplayNode: public rclcpp::Node {
    struct SetModeClient {
        SetModeClient(rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr p): ptr(p) {}
        std::atomic<bool> on_waiting = false;
        std::atomic<int> mode = -1;
        rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr ptr;
    };

public:
    AutoReplayNode(const rclcpp::NodeOptions& options):
        Node("auto_replay_node", rclcpp::NodeOptions(options).use_intra_process_comms(true)) {
        RCLCPP_INFO(this->get_logger(), "AutoReplayNode starting...");

        std::string record_dir =
            this->declare_parameter("record_dir", "/ros_ws/record/record_test");
        std::string target_frame = this->declare_parameter("target_frame", "odom");
        bool has_armor = this->declare_parameter("has_armor", true);
        bool has_rune = this->declare_parameter("has_rune", true);
        std::string camera_info_url = this->declare_parameter(
            "camera_info_url",
            "package://rm_bringup/config/camera_info.yaml"
        );
        video_hz_ = this->declare_parameter("video_hz", 60.0);
        serial_ahead_ms_ = this->declare_parameter("serial_ahead_ms", 10.0);

        image_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS());
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera_info",
            rclcpp::SensorDataQoS()
        );
        serial_receive_data_pub_ =
            this->create_publisher<rm_interfaces::msg::SerialReceiveData>("serial/receive", 10);

        heartbeat_ = qd::HeartBeatPublisher::create(this);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        video_records_ = readVideoTimestamps(record_dir + "/data_timestamps.csv");
        const bool has_video_timestamp = !video_records_.empty();
        const uint64_t min_video_timestamp =
            has_video_timestamp ? video_records_.front().timestamp : 0;
        serial_records_ = readSerialData(
            record_dir + "/data_serial.csv",
            min_video_timestamp,
            has_video_timestamp
        );
        video_cap_.open(record_dir + "/data.avi");

        if (!video_cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
        }

        // Camera Info
        camera_info_manager_ =
            std::make_shared<camera_info_manager::CameraInfoManager>(this, "narrow_stereo");
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_ = camera_info_manager_->getCameraInfo();
        camera_info_.header.frame_id = "camera_optical_frame";

        if (has_armor) {
            auto autoaim_set_mode_client_1 =
                this->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode");
            set_mode_clients_.emplace(
                autoaim_set_mode_client_1->get_service_name(),
                autoaim_set_mode_client_1
            );
            auto autoaim_set_mode_client_2 =
                this->create_client<rm_interfaces::srv::SetMode>("armor_solver/set_mode");
            set_mode_clients_.emplace(
                autoaim_set_mode_client_2->get_service_name(),
                autoaim_set_mode_client_2
            );
        }

        if (has_rune) {
            auto client1 =
                this->create_client<rm_interfaces::srv::SetMode>("rune_detector/set_mode");
            set_mode_clients_.emplace(client1->get_service_name(), client1);
            auto client2 = this->create_client<rm_interfaces::srv::SetMode>("rune_solver/set_mode");
            set_mode_clients_.emplace(client2->get_service_name(), client2);
        }

        target_frame_ = target_frame;

        if (!video_records_.empty() && !serial_records_.empty()) {
            start_clock_record_ =
                std::min(video_records_.front().timestamp, serial_records_.front().timestamp);
            play_start_time_ = this->now().nanoseconds();
        }

        replay_thread_ = std::thread(&AutoReplayNode::replayLoop, this);
    }

    ~AutoReplayNode() {
        if (replay_thread_.joinable()) {
            replay_thread_.join();
        }
    }

private:
    void setMode(SetModeClient& client, const uint8_t mode) {
        using namespace std::chrono_literals;

        std::string service_name = client.ptr->get_service_name();
        if (!client.ptr->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Service: %s is not available!", service_name.c_str());
            return;
        }
        auto req = std::make_shared<rm_interfaces::srv::SetMode::Request>();
        req->mode = mode;
        client.on_waiting.store(true);
        auto result = client.ptr->async_send_request(
            req,
            [mode, &client](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result) {
                client.on_waiting.store(false);
                if (result.get()->success) {
                    client.mode.store(mode);
                }
            }
        );
    }

    void publishSerialData(const SerialRecord& sr, uint64_t expected_sys_time) {
        auto expected_time_ros = rclcpp::Time(expected_sys_time);

        rm_interfaces::msg::SerialReceiveData serial_msg;
        serial_msg.header.stamp = expected_time_ros;
        serial_msg.header.frame_id = target_frame_;
        serial_msg.mode = sr.mode;
        serial_msg.bullet_speed = sr.bullet_speed;
        serial_msg.roll = sr.roll;
        serial_msg.pitch = sr.pitch;
        serial_msg.yaw = sr.yaw;

        serial_receive_data_pub_->publish(
            std::make_unique<rm_interfaces::msg::SerialReceiveData>(std::move(serial_msg))
        );
    }

    void publishSerialTf(const SerialRecord& sr, uint64_t expected_sys_time) {
        auto expected_time_ros = rclcpp::Time(expected_sys_time);

        geometry_msgs::msg::TransformStamped t_gimbal;
        t_gimbal.header.stamp = expected_time_ros;
        t_gimbal.header.frame_id = target_frame_;
        t_gimbal.child_frame_id = "gimbal_link";

        tf2::Quaternion q;
        q.setRPY(sr.roll * M_PI / 180.0, -sr.pitch * M_PI / 180.0, sr.yaw * M_PI / 180.0);
        t_gimbal.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t_gimbal);

        geometry_msgs::msg::TransformStamped t_rectify;

        // calculate RPY manually as utils::getRPY isn't imported
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        tf2::Quaternion q_rectify;
        q_rectify.setRPY(roll, 0, 0);

        t_rectify.header.stamp = expected_time_ros;
        t_rectify.header.frame_id = target_frame_;
        t_rectify.child_frame_id = "odom_rectify";
        t_rectify.transform.rotation = tf2::toMsg(q_rectify);
        tf_broadcaster_->sendTransform(t_rectify);
    }

    void replayLoop() {
        if (video_records_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No video data parsed, thread exit.");
            rclcpp::shutdown();
            return;
        }

        // 根据视频 Hz 计算帧间隔
        uint64_t frame_interval_ns = static_cast<uint64_t>(1e9 / video_hz_);
        // 计算串口提前发布的纳秒数
        uint64_t serial_ahead_ns = static_cast<uint64_t>(serial_ahead_ms_ * 1e6);

        size_t v_idx = 0;
        size_t s_idx = 0;
        uint8_t current_mode = 255;
        const SerialRecord* last_serial_record = nullptr;
        uint64_t last_frame_time = this->now().nanoseconds();

        while (rclcpp::ok() && v_idx < video_records_.size()) {
            // 按帧率等待
            uint64_t expected_time = last_frame_time + frame_interval_ns;
            uint64_t current_time = this->now().nanoseconds();
            if (expected_time > current_time) {
                std::this_thread::sleep_for(std::chrono::nanoseconds(expected_time - current_time));
            }
            last_frame_time = this->now().nanoseconds();

            const auto& vr = video_records_[v_idx];
            uint64_t bias = vr.timestamp - start_clock_record_;
            uint64_t expected_sys_time = play_start_time_ + bias;

            // 发布所有时间戳 <= 当前视频时间 - serial_ahead_ms 的串口消息
            if (!serial_records_.empty()) {
                uint64_t serial_target_timestamp = vr.timestamp - serial_ahead_ns;
                while (s_idx < serial_records_.size()) {
                    const auto& sr = serial_records_[s_idx];
                    if (sr.timestamp <= serial_target_timestamp) {
                        uint64_t serial_bias = sr.timestamp - start_clock_record_;
                        uint64_t serial_sys_time = play_start_time_ + serial_bias;
                        publishSerialData(sr, serial_sys_time);

                        // 检查mode变化
                        if (sr.mode != current_mode) {
                            current_mode = sr.mode;
                            for (auto& [service_name, client]: set_mode_clients_) {
                                if (client.mode.load() != current_mode && !client.on_waiting.load())
                                {
                                    setMode(client, current_mode);
                                }
                            }
                        }

                        last_serial_record = &sr;
                        s_idx++;
                    } else {
                        break;
                    }
                }

                // 仅在发布图像前发布最后一条串口记录的TF（减少TF流量）
                if (last_serial_record != nullptr) {
                    uint64_t serial_bias = last_serial_record->timestamp - start_clock_record_;
                    uint64_t serial_sys_time = play_start_time_ + serial_bias;
                    publishSerialTf(*last_serial_record, serial_sys_time);
                }
            }

            // 在发布图像前加一个小延迟，让TF有时间传播（2ms）
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            // 发布视频帧
            cv::Mat frame;
            if (video_cap_.read(frame) && !frame.empty()) {
                auto msg = std::make_unique<sensor_msgs::msg::Image>();
                auto expected_time_ros = rclcpp::Time(expected_sys_time);
                msg->header.stamp = expected_time_ros;
                msg->header.frame_id = "camera_optical_frame";
                msg->encoding = "bgr8";
                msg->height = frame.rows;
                msg->width = frame.cols;
                msg->step = frame.step;
                size_t size = frame.step * frame.rows;
                msg->data.resize(size);
                memcpy(msg->data.data(), frame.data, size);

                image_pub_->publish(std::move(msg));

                if (heartbeat_) {
                    heartbeat_->publish();
                }

                sensor_msgs::msg::CameraInfo camera_info_msg = camera_info_;
                camera_info_msg.header.stamp = expected_time_ros;
                if (camera_info_msg.width != static_cast<uint32_t>(frame.cols)) {
                    camera_info_msg.width = frame.cols;
                    camera_info_msg.height = frame.rows;
                }
                camera_info_pub_->publish(
                    std::make_unique<sensor_msgs::msg::CameraInfo>(std::move(camera_info_msg))
                );
            }
            v_idx++;
        }

        // 发布剩余的串口消息和TF
        while (s_idx < serial_records_.size()) {
            const auto& sr = serial_records_[s_idx];
            uint64_t serial_bias = sr.timestamp - start_clock_record_;
            uint64_t serial_sys_time = play_start_time_ + serial_bias;
            publishSerialData(sr, serial_sys_time);
            publishSerialTf(sr, serial_sys_time);

            if (sr.mode != current_mode) {
                current_mode = sr.mode;
                for (auto& [service_name, client]: set_mode_clients_) {
                    if (client.mode.load() != current_mode && !client.on_waiting.load()) {
                        setMode(client, current_mode);
                    }
                }
            }
            s_idx++;
        }

        RCLCPP_INFO(this->get_logger(), "Replay finished.");
        rclcpp::shutdown();
    }

    std::thread replay_thread_;
    std::string target_frame_;
    uint64_t start_clock_record_ = 0;
    uint64_t play_start_time_ = 0;
    double video_hz_ = 30.0;
    double serial_ahead_ms_ = 10.0;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_receive_data_pub_;

    std::unordered_map<std::string, SetModeClient> set_mode_clients_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    qd::HeartBeatPublisher::SharedPtr heartbeat_;

    std::vector<VideoRecord> video_records_;
    std::vector<SerialRecord> serial_records_;
    cv::VideoCapture video_cap_;

    sensor_msgs::msg::CameraInfo camera_info_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
};

} // namespace qd::auto_replay

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(qd::auto_replay::AutoReplayNode)
