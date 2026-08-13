#ifndef HIGH_PERF_DATA_RECORDER_HPP
#define HIGH_PERF_DATA_RECORDER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
// project
#include "rm_utils/logger/log.hpp"

class HighPerfDataRecorder {
public:
    HighPerfDataRecorder(std::string base_dir, double fps, int width, int height):
        fps_(fps),
        video_width_(width),
        video_height_(height),
        is_running_(true) {
        FYT_REGISTER_LOGGER("rm_recorder", "qd2026-log", INFO);

        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm parts;
        localtime_r(&now_c, &parts);

        std::ostringstream oss;
        oss << "record_" << std::setfill('0') << std::setw(4) << parts.tm_year + 1900 << "_"
            << std::setw(2) << parts.tm_mon + 1 << "_" << std::setw(2) << parts.tm_mday << "_"
            << std::setw(2) << parts.tm_hour << "_" << std::setw(2) << parts.tm_min << "_"
            << std::setw(2) << parts.tm_sec;

        std::filesystem::path dir_path = std::filesystem::path(base_dir) / oss.str();
        std::filesystem::create_directories(dir_path);

        std::string base_filename = (dir_path / "data").string();
        std::string video_filename = base_filename + ".avi";

        // 1. 视频通道初始化

        video_writer_.open(
            video_filename,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            fps,
            cv::Size(width, height),
            true
        );
        if (!video_writer_.isOpened()) {
            FYT_ERROR("rm_recorder", "cv::VideoWriter 无法打开！");
        }

        video_ts_file_.open(base_filename + "_timestamps.csv", std::ios::out | std::ios::trunc);
        if (video_ts_file_.is_open())
            video_ts_file_ << "FrameIndex,Timestamp\n";

        // 预分配视频内存池
        for (int i = 0; i < POOL_SIZE; ++i) {
            image_pool_.push_back({ cv::Mat(height, width, CV_8UC3), 0.0 });
            free_indices_.push(i);
        }

        // 2. 串口通道初始化
        serial_file_.open(base_filename + "_serial.csv", std::ios::out | std::ios::trunc);
        if (serial_file_.is_open()) {
            serial_file_ << "Timestamp,SerialData\n";
        }

        active_serial_buffer_.reserve(BATCH_SIZE);

        // 3. 启动工作线程
        video_thread_ = std::thread(&HighPerfDataRecorder::videoWorker, this);
        serial_thread_ = std::thread(&HighPerfDataRecorder::serialWorker, this);

        FYT_WARN(
            "rm_recorder",
            "init HighPerfDataRecorder with video: {}, serial: {}",
            video_writer_.isOpened() ? "ON" : "OFF",
            serial_file_.is_open() ? "ON" : "OFF"
        );
    }

    ~HighPerfDataRecorder() {
        is_running_ = false;

        // 确保退出前，将不满 BATCH_SIZE 的残余串口数据推入队列
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (!active_serial_buffer_.empty()) {
                serial_queue_.push(std::move(active_serial_buffer_));
            }
        }

        video_cv_.notify_one();
        serial_cv_.notify_one();

        if (video_thread_.joinable())
            video_thread_.join();
        if (serial_thread_.joinable())
            serial_thread_.join();

        if (video_writer_.isOpened())
            video_writer_.release();
        if (video_ts_file_.is_open())
            video_ts_file_.close();
        if (serial_file_.is_open())
            serial_file_.close();
    }

    void pushImage(const cv::Mat& frame, double timestamp) {
        if (!video_writer_.isOpened())
            return;

        // 根据目标帧率丢弃多余的帧 (timestamp 单位为纳秒)
        if (last_saved_timestamp_ > 0 && (timestamp - last_saved_timestamp_) < (1e9 / fps_)) {
            return;
        }
        last_saved_timestamp_ = timestamp;

        int idx = -1;
        // 步骤一：只在短临界区内获取空闲内存块的索引
        {
            std::lock_guard<std::mutex> lock(video_mutex_);
            if (free_indices_.empty()) {
                FYT_WARN("rm_recorder", "视频写入极度阻塞，触发防崩溃丢帧！");
                return;
            }
            idx = free_indices_.front();
            free_indices_.pop();
        }
        // 互斥锁在此处已释放，允许其他线程并行访问队列

        // 步骤二：在无锁状态下执行耗时的内存拷贝与颜色转换（外部输入为RGB，应转为OpenCV默认的BGR）
        // 处理因为外部输入尺寸不匹配导致 FFmpeg 写入被丢弃的情况
        if (frame.cols != video_width_ || frame.rows != video_height_) {
            cv::Mat resized;
            cv::resize(frame, resized, cv::Size(video_width_, video_height_));
            cv::cvtColor(resized, image_pool_[idx].frame, cv::COLOR_RGB2BGR);
        } else {
            cv::cvtColor(frame, image_pool_[idx].frame, cv::COLOR_RGB2BGR);
        }
        image_pool_[idx].timestamp = timestamp;

        // 步骤三：再次短加锁，将填满数据的索引推入就绪队列
        {
            std::lock_guard<std::mutex> lock(video_mutex_);
            ready_indices_.push(idx);
        }
        video_cv_.notify_one();
    }

    void pushSerialData(const std::string& data, double timestamp) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        active_serial_buffer_.push_back({ timestamp, data });

        if (active_serial_buffer_.size() >= BATCH_SIZE) {
            // 【核心修复】：直接将写满的 buffer 移动至队列，重新分配一个新的小内存块
            serial_queue_.push(std::move(active_serial_buffer_));
            active_serial_buffer_ = std::vector<SerialRecord>();
            active_serial_buffer_.reserve(BATCH_SIZE);
            serial_cv_.notify_one();
        }
    }

private:
    struct VideoFrame {
        cv::Mat frame;
        double timestamp;
    };
    struct SerialRecord {
        double timestamp;
        std::string data;
    };

    const int POOL_SIZE = 100;
    const size_t BATCH_SIZE = 100;
    double fps_;
    int video_width_;
    int video_height_;
    double last_saved_timestamp_ = -1.0;

    // 视频通道资源
    cv::VideoWriter video_writer_;
    std::ofstream video_ts_file_;
    std::vector<VideoFrame> image_pool_;
    std::queue<int> free_indices_;
    std::queue<int> ready_indices_;
    uint64_t frame_counter_ = 0;

    // 串口通道资源
    std::ofstream serial_file_;
    std::vector<SerialRecord> active_serial_buffer_;
    std::queue<std::vector<SerialRecord>> serial_queue_;

    // 并发控制
    std::atomic<bool> is_running_;
    std::mutex video_mutex_;
    std::condition_variable video_cv_;
    std::thread video_thread_;
    std::mutex serial_mutex_;
    std::condition_variable serial_cv_;
    std::thread serial_thread_;

    void videoWorker() {
        while (true) {
            int current_idx = -1;
            {
                std::unique_lock<std::mutex> lock(video_mutex_);
                video_cv_.wait(lock, [this]() { return !ready_indices_.empty() || !is_running_; });
                if (!is_running_ && ready_indices_.empty())
                    break;

                current_idx = ready_indices_.front();
                ready_indices_.pop();
            }

            if (current_idx != -1) {
                // 硬编落盘
                video_writer_.write(image_pool_[current_idx].frame);
                video_ts_file_ << std::fixed << frame_counter_++ << ","
                               << image_pool_[current_idx].timestamp << "\n";
                video_ts_file_.flush(); // 及时落盘，防止进程意外被死导致数据丢失

                // 归还内存块
                {
                    std::lock_guard<std::mutex> lock(video_mutex_);
                    free_indices_.push(current_idx);
                }
            }
        }
    }

    void serialWorker() {
        while (true) {
            std::vector<SerialRecord> buffer_to_write;
            {
                std::unique_lock<std::mutex> lock(serial_mutex_);
                serial_cv_.wait(lock, [this]() { return !serial_queue_.empty() || !is_running_; });
                if (!is_running_ && serial_queue_.empty())
                    break;

                buffer_to_write = std::move(serial_queue_.front());
                serial_queue_.pop();
            }

            // 无锁状态下落盘
            if (serial_file_.is_open() && !buffer_to_write.empty()) {
                for (const auto& record: buffer_to_write) {
                    serial_file_ << std::fixed << record.timestamp << "," << record.data << "\n";
                }
                serial_file_.flush(); // 及时落盘，防止进程意外被杀导致数据丢失
            }
        }
    }
};

#endif