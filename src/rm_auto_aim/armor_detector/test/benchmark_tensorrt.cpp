#include "armor_detector/tensorrt_inference.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

/**
 * @brief 测量后端同步调用耗时，包含拷贝、CUDA 预处理和模型执行。
 * @param inference 已预热且独占使用的后端实例。
 * @param image 已缩放补边的 RGB 图像。
 * @return 本次完整同步调用的毫秒数。
 */
double measure(qd::auto_aim::TensorRtInference& inference, const cv::Mat& image) {
    const auto start = std::chrono::steady_clock::now();
    inference.infer(image);
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
}

/**
 * @brief 将同步调用的延迟样本汇总成可保存的 JSON 字段。
 * @param name 输出的 JSON 字段名。
 * @param values 同步调用的毫秒延迟样本，必须非空。
 */
void print_summary(const char* name, std::vector<double> values) {
    std::sort(values.begin(), values.end());
    std::cout << '"' << name << "\":{\"samples\":" << values.size()
              << ",\"median_ms\":" << values[values.size() / 2]
              << ",\"p95_ms\":" << values[(values.size() - 1) * 95 / 100]
              << ",\"max_ms\":" << values.back() << '}';
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: benchmark_tensorrt <engine>\n";
        return 2;
    }
    try {
        qd::auto_aim::TensorRtInference regular(argv[1], false);
        qd::auto_aim::TensorRtInference captured(argv[1], true);
        cv::Mat image(640, 640, CV_8UC3);
        cv::RNG generator(20261119);
        generator.fill(image, cv::RNG::UNIFORM, 0, 256);
        for (int index = 0; index < 40; ++index) {
            regular.infer(image);
            captured.infer(image);
        }
        if (!captured.usesCudaGraph()) {
            throw std::runtime_error("CUDA graph capture failed; cannot compare graph timing");
        }
        std::vector<double> normal_times, graph_times;
        normal_times.reserve(200);
        graph_times.reserve(200);
        // 交替两种路径，减少温度和动态频率缓慢变化带来的顺序偏差。
        for (int index = 0; index < 200; ++index) {
            if (index % 2 == 0) {
                normal_times.push_back(measure(regular, image));
                graph_times.push_back(measure(captured, image));
            } else {
                graph_times.push_back(measure(captured, image));
                normal_times.push_back(measure(regular, image));
            }
        }
        std::cout << '{';
        print_summary("regular", std::move(normal_times));
        std::cout << ',';
        print_summary("cuda_graph", std::move(graph_times));
        std::cout << ",\"scope\":\"RGB640 to CPU output; includes copies, normalization, "
                     "TensorRT and synchronization; excludes decoding, resizing and YOLO parsing\"}\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
