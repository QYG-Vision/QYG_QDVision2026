#pragma once
// hikvision
#include "MvCameraControl.h"
// std
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
// opencv
#include <opencv2/opencv.hpp>
// project
#include "rm_utils/math/utils.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/thread_safe_queue.hpp"

namespace qd::Device {

class HikCamera {
public:
    struct CameraData {
        cv::Mat img;
        std::chrono::system_clock::time_point timestamp;
    };

    struct Parameters {
        double exposure_time; // 曝光时间
        double gain; // 增益
        double frame_rate; // 帧率
        bool enable_frame_rate; // 是否启用帧率控制
        std::string adc_bit_depth; // ADC位深
        std::string pixel_format; // 像素格式
        bool enable_DeviceSerialNumber; // 是否启用设备序列号
        std::string device_serial_number; // 设备序列号
        int cvt_quality; // 插值方法
        bool reverseXY; // 相机是否倒置安装
        bool enable_opencv; // 是否使用 OpenCV 进行 Bayer 插值
    };

    explicit HikCamera(const Parameters& params = Parameters());
    ~HikCamera();

    /**
     * @brief 开启守护线程
     * 
     */
    void starThread();

    /**
     * @brief Get the cv img object
     * 
     * @return cv::Mat 
     */
    cv::Mat get_cv_img();

    /**
     * @brief 直接调用sdk获得图像，不单独开启相机线程用
     * 
     * @param img 
     * @param timestamp 
     */
    void get_img(cv::Mat& img, std::chrono::system_clock::time_point& timestamp);

    /**
     * @brief 读取图像，需要开启守护线程
     * 
     * @param img 
     * @param timestamp 
     */
    void read_img(cv::Mat& img, std::chrono::system_clock::time_point& timestamp);

    /**
     * @brief 更新相机设置，需要修改 this->params_ 后调用
     * 
     */
    void update_camera_setting();

    /**
    * @brief 检查相机是否有效（句柄非空）
    * 
    * @return true 相机有效
    * @return false 相机无效（句柄为空）
    */
    bool isValid() const {
        return camera_handle_ != nullptr;
    }

public:
    Parameters params_; // 相机参数

private:
    /**
     * @brief 获取相机句柄，完成到取流环节
     * 
     * @param params 相机配置参数
     * @return void* 
     */
    void* get_camera_handle(const Parameters& params);

    /**
     * @brief 设置相机参数
     * 
     * @param handle 相机句柄
     * @param params 相机配置参数
     */
    void set_hik_param(void* handle, const Parameters& params);

    /**
     * @brief 图像回调函数
     * 
     */
    void image_callback();

    /**
     * @brief 获得相机曝光时间戳
     * 
     * @return std::chrono::system_clock::time_point 
     */
    std::chrono::system_clock::time_point get_time();

    /**
     * @brief 获得相机曝光时间戳
     * 
     * @param nHostTimeStamp  SDK 获得的图像 frame 里的时间戳，单位 ms
     * @return std::chrono::system_clock::time_point 
     */
    std::chrono::system_clock::time_point get_time(const int64_t& nHostTimeStamp);

private:
    tools::ThreadSafeQueue<CameraData> queue_; // 图像队列
    void* camera_handle_; // 相机句柄
    MV_FRAME_OUT out_frame_buffer_; // 图像缓存
    MV_CC_PIXEL_CONVERT_PARAM convert_param; // 像素格式转换参数结构体

    int nRet = MV_OK; // 函数返回值
    std::chrono::system_clock::duration half_exposure_duration; // 相机曝光一半时间

    std::thread daemon_thread_; // 守护线程
    std::atomic<bool> running_;
};

} // namespace qd::Device
