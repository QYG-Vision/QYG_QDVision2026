#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "armor_detector/yolov5_detector.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/url_resolver.hpp"

namespace qd::auto_aim {
namespace {

    const Yolov5Detector::Yolov5Params PARAMS = { 0.65, 0.45, 3.5, { "negative" } };

    /**
     * @brief 使用仓库原模型构造 CPU 基准检测器。
     * @return 已加载模型的 OpenVINO CPU 检测器。
     */
    std::unique_ptr<Yolov5Detector> make_cpu_detector() {
        FYT_REGISTER_LOGGER("armor_detector", "/tmp/qyg-armor-tests", INFO);
        return std::make_unique<Yolov5Detector>(
            utils::URLResolver::getResolvedPath("package://armor_detector/model/IR/0526.xml")
                .string(),
            utils::URLResolver::getResolvedPath("package://armor_detector/model/IR/0526.bin")
                .string(),
            "CPU",
            EnemyColor::RED,
            PARAMS
        );
    }

    /**
     * @brief 对比两个后端的最终检测，容差按原图像素和置信度定义。
     */
    void compare_detections(std::vector<Armor> reference, std::vector<Armor> actual) {
        const auto order = [](const Armor& left, const Armor& right) {
            return left.center.x < right.center.x;
        };
        std::sort(reference.begin(), reference.end(), order);
        std::sort(actual.begin(), actual.end(), order);
        ASSERT_EQ(actual.size(), reference.size());
        for (size_t index = 0; index < reference.size(); ++index) {
            EXPECT_EQ(actual[index].number, reference[index].number);
            EXPECT_EQ(actual[index].type, reference[index].type);
            EXPECT_NEAR(actual[index].confidence, reference[index].confidence, 0.02);
            const auto expected_corners = reference[index].landmarks();
            const auto actual_corners = actual[index].landmarks();
            for (size_t corner = 0; corner < expected_corners.size(); ++corner) {
                EXPECT_LT(cv::norm(actual_corners[corner] - expected_corners[corner]), 2.0);
            }
        }
    }

    TEST(YoloBackends, RejectsUnknownBackend) {
        EXPECT_THROW(
            Yolov5Detector("", "", "CPU", EnemyColor::RED, PARAMS, "unknown"),
            std::invalid_argument
        );
    }

    TEST(YoloBackends, CpuLoadsExistingIrAndRejectsInvalidImages) {
        auto detector = make_cpu_detector();
        EXPECT_THROW(detector->detect(cv::Mat()), std::invalid_argument);
        EXPECT_THROW(detector->detect(cv::Mat::zeros(100, 200, CV_8UC1)), std::invalid_argument);
        EXPECT_NO_THROW(detector->detect(cv::Mat::zeros(480, 640, CV_8UC3)));
    }

    TEST(YoloBackends, TensorRtMatchesCpuOnReferenceImage) {
        const char* engine_path = std::getenv("ARMOR_TENSORRT_ENGINE");
        if (engine_path == nullptr || engine_path[0] == '\0') {
            GTEST_SKIP() << "Set ARMOR_TENSORRT_ENGINE on the Jetson to run GPU parity validation";
        }
        auto cpu = make_cpu_detector();
        Yolov5Detector gpu(engine_path, "", "", EnemyColor::RED, PARAMS, "tensorrt");
        auto image = cv::imread(
            utils::URLResolver::getResolvedPath("package://armor_detector/docs/test.png").string()
        );
        ASSERT_FALSE(image.empty());
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        auto reference = cpu->detect(image);
        auto actual = gpu.detect(image);
        ASSERT_FALSE(reference.empty());
        compare_detections(reference, actual);
    }

    TEST(YoloBackends, TensorRtMatchesCpuOnVideoSamples) {
        const char* engine = std::getenv("ARMOR_TENSORRT_ENGINE");
        const char* video = std::getenv("ARMOR_TEST_VIDEO");
        if (engine == nullptr || video == nullptr) {
            GTEST_SKIP() << "Set ARMOR_TENSORRT_ENGINE and ARMOR_TEST_VIDEO";
        }
        auto cpu = make_cpu_detector();
        Yolov5Detector gpu(engine, "", "", EnemyColor::RED, PARAMS, "tensorrt");
        cv::VideoCapture capture(video);
        ASSERT_TRUE(capture.isOpened());
        const auto frames = capture.get(cv::CAP_PROP_FRAME_COUNT);
        size_t detections = 0;
        for (const double fraction : {0.0, 0.2, 0.4, 0.6, 0.8}) {
            SCOPED_TRACE(fraction);
            capture.set(cv::CAP_PROP_POS_FRAMES, (frames - 1) * fraction);
            cv::Mat image;
            ASSERT_TRUE(capture.read(image));
            cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
            for (const auto color : {EnemyColor::RED, EnemyColor::BLUE}) {
                cpu->detect_color = color;
                gpu.detect_color = color;
                const auto reference = cpu->detect(image);
                detections += reference.size();
                compare_detections(reference, gpu.detect(image));
            }
        }
        EXPECT_GT(detections, 0U);
    }

    TEST(YoloBackends, CudaGraphReplaysCurrentInputAndMatchesRegularEnqueue) {
        const char* engine = std::getenv("ARMOR_TENSORRT_ENGINE");
        if (engine == nullptr) {
            GTEST_SKIP() << "Set ARMOR_TENSORRT_ENGINE";
        }
        TensorRtInference regular(engine, false);
        TensorRtInference captured(engine, true);
        cv::Mat random(640, 640, CV_8UC3);
        cv::RNG generator(20261119);
        generator.fill(random, cv::RNG::UNIFORM, 0, 256);
        for (const cv::Mat& image : {cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0)), random,
                                    cv::Mat(640, 640, CV_8UC3, cv::Scalar(255, 127, 0))}) {
            const auto reference = regular.infer(image);
            const auto actual = captured.infer(image);
            ASSERT_TRUE(captured.usesCudaGraph());
            EXPECT_FALSE(regular.usesCudaGraph());
            EXPECT_LE(cv::norm(reference, actual, cv::NORM_INF), 1e-3);
        }
    }

} // namespace
} // namespace qd::auto_aim
