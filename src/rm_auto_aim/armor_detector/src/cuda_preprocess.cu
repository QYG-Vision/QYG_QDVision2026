#include "armor_detector/cuda_preprocess.hpp"

namespace qd::auto_aim {
namespace {

__global__ void normalizeRgbKernel(const std::uint8_t* rgb, float* nchw, std::size_t pixels) {
    const std::size_t index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index < pixels) {
        constexpr float SCALE = 1.0F / 255.0F;
        nchw[index] = static_cast<float>(rgb[index * 3]) * SCALE;
        nchw[pixels + index] = static_cast<float>(rgb[index * 3 + 1]) * SCALE;
        nchw[pixels * 2 + index] = static_cast<float>(rgb[index * 3 + 2]) * SCALE;
    }
}

} // namespace

cudaError_t normalizeRgbCuda(
    const std::uint8_t* rgb,
    float* nchw,
    std::size_t pixels,
    cudaStream_t stream
) {
    if (pixels == 0 || rgb == nullptr || nchw == nullptr) {
        return cudaErrorInvalidValue;
    }
    constexpr unsigned int THREADS = 256;
    const auto blocks = static_cast<unsigned int>((pixels + THREADS - 1) / THREADS);
    normalizeRgbKernel<<<blocks, THREADS, 0, stream>>>(rgb, nchw, pixels);
    return cudaGetLastError();
}

} // namespace qd::auto_aim
