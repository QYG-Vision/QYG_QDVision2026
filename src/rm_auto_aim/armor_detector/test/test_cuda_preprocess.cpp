#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include "armor_detector/cuda_preprocess.hpp"

namespace qd::auto_aim {
namespace {

class CudaPreprocessTest : public ::testing::Test {
protected:
    void TearDown() override {
        cudaFree(rgb_);
        cudaFree(output_);
    }
    std::uint8_t* rgb_ = nullptr;
    float* output_ = nullptr;
};

TEST_F(CudaPreprocessTest, MatchesOpenCvForPartialBlockAndAllChannels) {
    int devices = 0;
    ASSERT_EQ(cudaGetDeviceCount(&devices), cudaSuccess);
    ASSERT_GT(devices, 0);
    // 非线程块整数倍的尺寸，覆盖末尾边界和 RGB 通道顺序。
    cv::Mat image(17, 19, CV_8UC3);
    cv::RNG random(20261119);
    random.fill(image, cv::RNG::UNIFORM, 0, 256);
    image.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 127, 255);
    const auto reference = cv::dnn::blobFromImage(image, 1.0 / 255.0);
    const size_t pixels = image.total();
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&rgb_), pixels * 3), cudaSuccess);
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&output_), pixels * 3 * sizeof(float)),
              cudaSuccess);
    ASSERT_EQ(cudaMemcpy(rgb_, image.data, pixels * 3, cudaMemcpyHostToDevice), cudaSuccess);
    ASSERT_EQ(normalizeRgbCuda(rgb_, output_, pixels, nullptr), cudaSuccess);
    cv::Mat actual(reference.dims, reference.size, CV_32F);
    ASSERT_EQ(cudaMemcpy(actual.data, output_, pixels * 3 * sizeof(float),
                         cudaMemcpyDeviceToHost), cudaSuccess);
    EXPECT_LT(cv::norm(reference, actual, cv::NORM_INF), 1e-6);
    EXPECT_EQ(normalizeRgbCuda(nullptr, output_, pixels, nullptr), cudaErrorInvalidValue);
    EXPECT_EQ(normalizeRgbCuda(rgb_, output_, 0, nullptr), cudaErrorInvalidValue);
}

} // namespace
} // namespace qd::auto_aim
