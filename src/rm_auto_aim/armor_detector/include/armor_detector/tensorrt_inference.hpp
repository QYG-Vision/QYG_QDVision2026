#ifndef ARMOR_DETECTOR_TENSORRT_INFERENCE_HPP_
#define ARMOR_DETECTOR_TENSORRT_INFERENCE_HPP_

#include <memory>
#include <opencv2/core.hpp>
#include <string>

namespace qd::auto_aim {

class TensorRtInference {
public:
    /**
     * @brief 加载 TensorRT 10 的装甲板模型并分配推理资源。
     * @param engine_path 在目标设备生成的 engine 文件路径。
     * @param use_cuda_graph 是否在首帧预热后捕获固定执行图，失败时沿用普通推理。
     * @note 仅接受 FP32、线性布局的 [1,3,640,640] 输入和 [1,N,22] 输出。
     */
    explicit TensorRtInference(const std::string& engine_path, bool use_cuda_graph = true);
    ~TensorRtInference();

    TensorRtInference(const TensorRtInference&) = delete;
    TensorRtInference& operator=(const TensorRtInference&) = delete;

    /**
     * @brief 对已缩放及补黑边的 RGB 图像执行同步推理。
     * @param rgb_image 640×640 的 CV_8UC3 图像，不交换通道，归一化到 [0,1]。
     * @return N×22 的 FP32 预测；底层内存由本对象持有，下次推理时覆盖。
     * @note 同一实例不能并发调用；CUDA 资源始终在创建时的设备上使用。
     */
    cv::Mat infer(const cv::Mat& rgb_image);

    /** @brief 返回是否已成功捕获并可复用 CUDA 执行图。 */
    bool usesCudaGraph() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace qd::auto_aim

#endif // ARMOR_DETECTOR_TENSORRT_INFERENCE_HPP_
