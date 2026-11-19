#ifndef ARMOR_DETECTOR_CUDA_PREPROCESS_HPP_
#define ARMOR_DETECTOR_CUDA_PREPROCESS_HPP_

#include <cuda_runtime_api.h>
#include <cstddef>
#include <cstdint>

namespace qd::auto_aim {

/**
 * @brief 在调用方的 CUDA 流上将 RGB uint8 HWC 转为归一化 FP32 NCHW。
 * @param rgb 设备端连续 RGB 字节数组，颜色和缩放补边已由上游确定。
 * @param nchw 设备端输出，容量至少为 pixels * 3 个 float。
 * @param pixels 像素数，必须大于零。
 * @param stream 与 TensorRT 推理共用的 CUDA 流。
 * @return 内核启动状态；异步执行错误由调用方同步流时检查。
 */
cudaError_t normalizeRgbCuda(
    const std::uint8_t* rgb,
    float* nchw,
    std::size_t pixels,
    cudaStream_t stream
);

} // namespace qd::auto_aim

#endif
