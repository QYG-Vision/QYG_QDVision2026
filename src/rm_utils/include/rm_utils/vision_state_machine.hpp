#ifndef RM_UTILS_VISION_STATE_MACHINE_HPP_
#define RM_UTILS_VISION_STATE_MACHINE_HPP_
// ROS2
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/armor.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
// Third party
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
// std
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
// Project
#include "rm_utils/common.hpp"
#include "rm_utils/driver/HikCamera.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/thread_safe_state.hpp"
#include "rm_utils/url_resolver.hpp"

namespace qd::utils {

using hikParame = qd::Device::HikCamera::Parameters;

/**
 * @brief 相机类型枚举
 */
enum class CameraType {
    CAMERA_NEAR_RANGE = 0, // 近距离相机
    CAMERA_FAR_RANGE = 1, // 远距离相机
};

/**
 * @brief 相机配置参数结构
 * 基于 HikCamera::Parameters，并添加额外的ROS相关配置
 */
struct CameraConfig {
    hikParame hik_params; // 海康相机参数

    // ROS相关配置
    std::string camera_info_url; // 相机内参文件路径
    std::string camera_name; // 相机名称
    std::string target_frame; // 目标frame_id

    // 相机切换阈值
    double switch_to_8mm_distance; // 切换到远距离相机的距离阈值
    double switch_to_6mm_distance; // 切换回近距离相机的距离阈值
};

/**
 * @brief 自瞄检测链路共享帧，仅保留最新一帧。
 */
struct ArmorFrame {
    std_msgs::msg::Header header;
    std::vector<rm_interfaces::msg::Armor> armors;
    cv::Mat image;
    sensor_msgs::msg::CameraInfo camera_info;
    std::uint64_t sequence { 0 };
};

/**
 * @brief 视觉状态机（静态单例）
 * 
 * 功能：
 * 1. 管理双相机（近距离和远距离）
 * 2. 管理模式切换（VisionMode）
 * 3. 管理跟踪状态（TrackerState）
 * 4. 管理相机切换逻辑
 * 5. 线程安全的相机内参访问
 */
class VisionStateMachine {
public:
    /**
     * @brief 获取单例实例
     */
    static VisionStateMachine& getInstance();

    /**
     * @brief 初始化海康相机参数（需要ROS2节点指针）
     * @param node ROS2节点指针
     * @param config_file 配置文件路径（可选），有的话使用yaml-cpp读配置
     */
    bool initialize(rclcpp::Node* node, const std::string& config_file = "");

    /**
     * @brief 初始化相机
     * @param camera_type 相机类型
     * @return 是否成功
     */
    bool initializeCamera(CameraType camera_type);

    /**
     * @brief 设置视觉识别模式
     * @param mode 视觉模式
     */
    void setVisionMode(VisionMode mode);

    /**
     * @brief 获取当前视觉识别模式
     */
    VisionMode getVisionMode() const;

    /**
     * @brief 设置自瞄跟踪器状态
     * @param state 跟踪器状态
     */
    void setTrackerState(TrackerState state);

    /**
     * @brief 获取当前跟踪器状态
     */
    TrackerState getTrackerState() const;

    /**
     * @brief 更新目标距离（用于相机切换判断）
     * @param distance 目标距离（米）
     */
    void updateTargetDistance(double distance);

    /**
     * @brief 更新相机切换逻辑
     */
    void updateCameraSwitchLogic();

    /**
     * @brief 获取当前图像（线程安全）
     * @param image 输出图像
     * @param camera_info 输出相机内参
     * @return 是否成功获取
     */
    bool getCurrentImage(sensor_msgs::msg::Image& image, sensor_msgs::msg::CameraInfo& camera_info);

    /**
     * @brief 获取当前激活相机的一帧图像、内参和时间戳。
     * @param img 输出图像，像素格式与当前相机配置一致。
     * @param camera_info 输出相机内参，header 会同步到当前帧。
     * @param timestamp 输出图像曝光中点对应的系统时钟时间戳。
     * @return 成功获取当前帧时返回 true，否则返回 false。
     */
    bool getCurrentImage(
        cv::Mat& img,
        sensor_msgs::msg::CameraInfo& camera_info,
        std::chrono::system_clock::time_point& timestamp
    );

    void getCurrentImage(cv::Mat& img, std::chrono::system_clock::time_point& timestamp);

    /**
     * @brief 获取当前相机内参（线程安全），ros相关
     * @param camera_info 输出相机内参
     * @return 是否成功获取
     */
    bool getCurrentCameraInfo(sensor_msgs::msg::CameraInfo& camera_info);

    /**
     * @brief 获取当前相机的OpenCV格式相机参数（线程安全），用于solvepnp
     * @param camera_matrix 输出相机内参矩阵 (3x3)
     * @param distortion_coefficients 输出畸变系数 (1x5)
     * @return 是否成功获取
     */
    bool getCurrentCameraParams(cv::Mat& camera_matrix, cv::Mat& distortion_coefficients);

    /**
     * @brief 发布 detector 到 solver 的共享帧，仅保留最新结果。
     * @param header 当前检测结果对应的消息头。
     * @param armors 当前帧解算后的装甲板结果。
     * @param image 当前帧图像，调用后所有权移交给状态机。
     * @param camera_info 当前帧对应的相机内参。
     */
    void publishArmorFrame(
        const std_msgs::msg::Header& header,
        std::vector<rm_interfaces::msg::Armor>&& armors,
        cv::Mat&& image,
        sensor_msgs::msg::CameraInfo&& camera_info
    );

    /**
     * @brief 阻塞读取最新一帧 detector 共享结果，并将其从槽中取出。
     * @param armor_frame 输出共享帧。
     * @param timeout 最长等待时长。
     * @return 读取到共享帧时返回 true，超时返回 false。
     */
    bool waitAndPopLatestArmorFrame(
        ArmorFrame& armor_frame,
        std::chrono::milliseconds timeout = std::chrono::milliseconds(50)
    );

    /**
     * @brief 唤醒所有阻塞在 waitAndPopLatestArmorFrame 上的线程，便于节点优雅关闭。
     */
    void notifyArmorFrameWaiters();

    /**
     * @brief 获取当前使用的相机类型
     */
    CameraType getCurrentCameraType() const;

    /**
     * @brief 获取当前相机的target_frame（线程安全）
     * @return 当前相机的target_frame字符串，如果相机未初始化则返回空字符串
     */
    std::string getCurrentTargetFrame() const;

    /**
     * @brief 检查是否应该进行自瞄识别
     */
    bool shouldDetect() const;

    /**
     * @brief 更新当前相机的曝光时间
     * @param exposure_time 曝光时间（微秒）
     */
    void updateExposureTime(double exposure_time);

    /**
     * @brief 更新当前相机的增益
     * @param gain 增益值
     */
    void updateGain(double gain);

    /**
     * @brief 析构函数
     */
    ~VisionStateMachine();

    // 禁止拷贝和赋值
    VisionStateMachine(const VisionStateMachine&) = delete;
    VisionStateMachine& operator=(const VisionStateMachine&) = delete;

private:
    VisionStateMachine() = default;

    /**
     * @brief 关闭相机
     * @param camera_type 相机类型
     */
    void closeCamera(CameraType camera_type);

    /**
     * @brief 切换相机
     * @param target_type 目标相机类型
     */
    void switchCamera(CameraType target_type);

    /**
     * @brief 相机采集线程
     */
    void cameraCaptureThread(CameraType camera_type);

    /**
     * @brief 加载相机配置
     * @param config_file 配置文件路径
     */
    bool loadConfig(const std::string& config_file);

    /**
     * @brief 加载ros发布相机内参消息，ros相关
     * 
     * @param camera_type 
     * @param config 
     */
    void load_camera_info_managers(CameraType camera_type, const CameraConfig& config);

    // 单例实例
    static std::unique_ptr<VisionStateMachine> instance_;
    static std::mutex instance_mutex_;

    // ROS2节点指针
    rclcpp::Node* node_ { nullptr };

    // 状态变量（线程安全）
    ThreadSafeState<VisionMode> current_vision_mode_ { VisionMode::AUTO_AIM_RED }; // 视觉识别模式
    ThreadSafeState<TrackerState> current_tracker_state_ { TrackerState::LOST }; // 自瞄跟踪器状态
    ThreadSafeState<CameraType> current_camera_type_ {
        CameraType::CAMERA_NEAR_RANGE
    }; // 使用的相机类型
    ThreadSafeState<double> last_target_distance_ { 0.0 }; // 目标距离

    // 相机配置
    std::map<CameraType, CameraConfig> camera_configs_;

    // 相机信息管理器
    std::map<CameraType, std::unique_ptr<camera_info_manager::CameraInfoManager>>
        camera_info_managers_;

    // 当前图像和相机信息（线程安全）
    std::mutex image_mutex_;
    sensor_msgs::msg::Image current_image_;
    sensor_msgs::msg::CameraInfo current_camera_info_;
    bool image_available_ { false };

    // detector -> solver latest-only 共享结果槽
    std::mutex armor_frame_mutex_;
    std::condition_variable armor_frame_cv_;
    std::optional<ArmorFrame> latest_armor_frame_;
    std::uint64_t armor_frame_sequence_ { 0 };

    // 相机采集线程
    std::map<CameraType, std::thread> capture_threads_;
    std::map<CameraType, std::atomic<bool>> camera_running_;

    // 相机
    std::map<CameraType, qd::Device::HikCamera> hik_camera_;

    // 状态机互斥锁
    mutable std::mutex state_mutex_;
};

} // namespace qd::utils

#endif // RM_UTILS_VISION_STATE_MACHINE_HPP_
