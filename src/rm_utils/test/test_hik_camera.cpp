#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "rm_utils/driver/HikCamera.hpp"
#include "rm_utils/vision_state_machine.hpp"
#include "gtest/gtest.h"

namespace {

struct FakeSdkState {
    int enum_result { MV_OK };
    int open_result { MV_OK };
    int start_result { MV_OK };
    int serial_result { MV_OK };
    int grab_result { MV_OK };
    int convert_result { MV_OK };
    bool incomplete_conversion { false };
    unsigned int conversion_capacity { 0 };
    int freed_buffers { 0 };
    MV_FRAME_OUT_INFO_EX frame_info {};
    std::vector<unsigned char> pixels;
    std::vector<void*> destroyed_handles;
    std::function<void()> on_start;
};

FakeSdkState sdk;
int fake_handle;
MV_CC_DEVICE_INFO fake_device {};

class HikCameraTest: public ::testing::Test {
protected:
    /** @brief 复位模拟 SDK，保证测试不打开真实相机。 */
    void SetUp() override {
        sdk = FakeSdkState {};
        sdk.frame_info.nWidth = sdk.frame_info.nExtendWidth = 8;
        sdk.frame_info.nHeight = sdk.frame_info.nExtendHeight = 8;
        sdk.frame_info.nHostTimeStamp = 1700000000000LL;
        sdk.frame_info.enPixelType = PixelType_Gvsp_RGB8_Packed;
        sdk.pixels.assign(8 * 8 * 3, 42);
        sdk.frame_info.nFrameLen = sdk.pixels.size();
        params_.exposure_time = 1000.0;
        params_.gain = 1.0;
        params_.frame_rate = 100.0;
        params_.enable_frame_rate = true;
        params_.adc_bit_depth = "ADCBitDepth_8";
        params_.pixel_format = "BayerRG8";
        params_.cvt_quality = 2;
    }

    qd::Device::HikCamera::Parameters params_ {};
};

TEST_F(HikCameraTest, destroys_device_handle_itself_on_normal_shutdown) {
    {
        qd::Device::HikCamera camera(params_);
        EXPECT_TRUE(camera.isValid());
    }
    ASSERT_EQ(sdk.destroyed_handles.size(), 1U);
    EXPECT_EQ(sdk.destroyed_handles.front(), &fake_handle);
}

TEST_F(HikCameraTest, destroys_probe_and_selected_handles_after_serial_number_match) {
    params_.enable_DeviceSerialNumber = true;
    params_.device_serial_number = "TESTCAM";
    {
        qd::Device::HikCamera camera(params_);
        EXPECT_TRUE(camera.isValid());
    }
    ASSERT_EQ(sdk.destroyed_handles.size(), 2U);
    EXPECT_EQ(sdk.destroyed_handles[0], &fake_handle);
    EXPECT_EQ(sdk.destroyed_handles[1], &fake_handle);
}

TEST_F(HikCameraTest, destroys_probe_handle_when_open_fails) {
    params_.enable_DeviceSerialNumber = true;
    params_.device_serial_number = "TESTCAM";
    sdk.open_result = MV_E_HANDLE;
    qd::Device::HikCamera camera(params_);
    EXPECT_FALSE(camera.isValid());
    ASSERT_EQ(sdk.destroyed_handles.size(), 1U);
    EXPECT_EQ(sdk.destroyed_handles.front(), &fake_handle);
}

TEST_F(HikCameraTest, destroys_selected_handle_when_open_fails) {
    sdk.open_result = MV_E_HANDLE;
    qd::Device::HikCamera camera(params_);
    EXPECT_FALSE(camera.isValid());
    ASSERT_EQ(sdk.destroyed_handles.size(), 1U);
    EXPECT_EQ(sdk.destroyed_handles.front(), &fake_handle);
}

TEST_F(HikCameraTest, destroys_selected_handle_when_start_grabbing_fails) {
    sdk.start_result = MV_E_HANDLE;
    qd::Device::HikCamera camera(params_);
    EXPECT_FALSE(camera.isValid());
    ASSERT_EQ(sdk.destroyed_handles.size(), 1U);
    EXPECT_EQ(sdk.destroyed_handles.front(), &fake_handle);
}

TEST_F(HikCameraTest, rejects_failed_device_enumeration) {
    sdk.enum_result = MV_E_HANDLE;
    qd::Device::HikCamera camera(params_);
    EXPECT_FALSE(camera.isValid());
    EXPECT_TRUE(sdk.destroyed_handles.empty());
}

TEST_F(HikCameraTest, rejects_failed_serial_query_instead_of_matching_its_output) {
    params_.enable_DeviceSerialNumber = true;
    params_.device_serial_number = "TESTCAM";
    sdk.serial_result = MV_E_HANDLE;
    qd::Device::HikCamera camera(params_);
    EXPECT_FALSE(camera.isValid());
    EXPECT_EQ(sdk.destroyed_handles.size(), 1U);
}

TEST_F(HikCameraTest, converts_into_exact_rgb_capacity_and_owns_output_after_buffer_release) {
    qd::Device::HikCamera camera(params_);
    cv::Mat image;
    std::chrono::system_clock::time_point timestamp;
    camera.get_img(image, timestamp);
    ASSERT_FALSE(image.empty());
    EXPECT_EQ(image.type(), CV_8UC3);
    EXPECT_EQ(sdk.conversion_capacity, 8U * 8U * 3U);
    EXPECT_EQ(sdk.freed_buffers, 1);
    EXPECT_EQ(image.at<cv::Vec3b>(0, 0), cv::Vec3b(42, 42, 42));
    EXPECT_NE(timestamp, std::chrono::system_clock::time_point {});
}

TEST_F(HikCameraTest, falls_back_to_sdk_for_non_bayer_format_when_opencv_enabled) {
    params_.enable_opencv = true;
    qd::Device::HikCamera camera(params_);
    cv::Mat image;
    std::chrono::system_clock::time_point timestamp;
    EXPECT_NO_THROW(camera.get_img(image, timestamp));
    ASSERT_FALSE(image.empty());
    EXPECT_EQ(sdk.conversion_capacity, 8U * 8U * 3U);
    EXPECT_EQ(sdk.freed_buffers, 1);
}

TEST_F(HikCameraTest, returns_rgb_for_all_supported_bayer_patterns) {
    params_.enable_opencv = true;
    const std::vector<MvGvspPixelType> patterns {
        PixelType_Gvsp_BayerRG8, PixelType_Gvsp_BayerGR8,
        PixelType_Gvsp_BayerGB8, PixelType_Gvsp_BayerBG8
    };
    // 每个 2x2 Bayer 单元中红、蓝采样点的位置。
    const std::vector<int> red_positions {0, 1, 2, 3};
    const std::vector<int> blue_positions {3, 2, 1, 0};
    qd::Device::HikCamera camera(params_);
    for (size_t p = 0; p < patterns.size(); ++p) {
        sdk.frame_info.enPixelType = patterns[p];
        sdk.frame_info.nFrameLen = 64;
        sdk.pixels.resize(64);
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                const int position = (y % 2) * 2 + x % 2;
                sdk.pixels[y * 8 + x] = position == red_positions[p] ? 200
                    : position == blue_positions[p] ? 20 : 100;
            }
        }
        cv::Mat image;
        std::chrono::system_clock::time_point timestamp;
        ASSERT_NO_THROW(camera.get_img(image, timestamp));
        ASSERT_FALSE(image.empty());
        EXPECT_EQ(image.at<cv::Vec3b>(3, 3), cv::Vec3b(200, 100, 20));
    }
    EXPECT_EQ(sdk.freed_buffers, 4);
    EXPECT_EQ(sdk.conversion_capacity, 0U);
}

TEST_F(HikCameraTest, drops_failed_or_incomplete_conversion_and_releases_sdk_buffer) {
    qd::Device::HikCamera camera(params_);
    for (bool incomplete : {false, true}) {
        sdk.convert_result = incomplete ? MV_OK : MV_E_HANDLE;
        sdk.incomplete_conversion = incomplete;
        cv::Mat image(2, 2, CV_8UC3, cv::Scalar(1, 2, 3));
        auto timestamp = std::chrono::system_clock::now();
        camera.get_img(image, timestamp);
        EXPECT_TRUE(image.empty());
        EXPECT_EQ(timestamp, std::chrono::system_clock::time_point {});
    }
    EXPECT_EQ(sdk.freed_buffers, 2);
}

TEST_F(HikCameraTest, drops_truncated_bayer_and_releases_sdk_buffer) {
    params_.enable_opencv = true;
    sdk.frame_info.enPixelType = PixelType_Gvsp_BayerRG8;
    sdk.frame_info.nFrameLen = 1;
    qd::Device::HikCamera camera(params_);
    cv::Mat image;
    std::chrono::system_clock::time_point timestamp;
    camera.get_img(image, timestamp);
    EXPECT_TRUE(image.empty());
    EXPECT_EQ(sdk.freed_buffers, 1);
}

TEST_F(HikCameraTest, drops_invalid_dimensions_and_releases_sdk_buffer) {
    sdk.frame_info.nWidth = 0;
    qd::Device::HikCamera camera(params_);
    cv::Mat image;
    std::chrono::system_clock::time_point timestamp;
    camera.get_img(image, timestamp);
    EXPECT_TRUE(image.empty());
    EXPECT_EQ(sdk.freed_buffers, 1);
}

TEST_F(HikCameraTest, clears_previous_output_when_grabbing_fails) {
    sdk.grab_result = MV_E_HANDLE;
    qd::Device::HikCamera camera(params_);
    cv::Mat image(2, 2, CV_8UC3, cv::Scalar(1, 2, 3));
    auto timestamp = std::chrono::system_clock::now();
    camera.get_img(image, timestamp);
    EXPECT_TRUE(image.empty());
    EXPECT_EQ(timestamp, std::chrono::system_clock::time_point {});
    EXPECT_EQ(sdk.freed_buffers, 0);
}

TEST_F(HikCameraTest, updates_exposure_midpoint_snapshot_with_camera_settings) {
    qd::Device::HikCamera camera(params_);
    cv::Mat image;
    std::chrono::system_clock::time_point before, after;
    camera.get_img(image, before);
    camera.params_.exposure_time = 2000.0;
    camera.update_camera_setting();
    camera.get_img(image, after);
    EXPECT_EQ(before - after, std::chrono::microseconds(500));
}

TEST_F(HikCameraTest, stops_initialization_if_context_shuts_down_while_opening_camera) {
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);
    options.parameter_overrides({ rclcpp::Parameter("camera_6mm.enable_DeviceSerialNumber", false) }
    );
    auto node = std::make_shared<rclcpp::Node>("test_hik_camera_shutdown", options);
    auto& state_machine = qd::utils::VisionStateMachine::getInstance();
    ASSERT_TRUE(state_machine.initialize(node.get()));
    sdk.on_start = [context]() { context->shutdown("simulate Ctrl+C during camera open"); };

    bool initialized = true;
    EXPECT_NO_THROW(
        initialized = state_machine.initializeCamera(qd::utils::CameraType::CAMERA_NEAR_RANGE)
    );
    EXPECT_FALSE(initialized);
    ASSERT_EQ(sdk.destroyed_handles.size(), 1U);
    EXPECT_EQ(sdk.destroyed_handles.front(), &fake_handle);
    sdk.on_start = {};
    state_machine.releaseCameraResources(node.get());
}

} // namespace

// 使用与 SDK 头文件相同的 C 接口替身，检查实际传入的句柄并隔离真实硬件。
extern "C" {

int MV_CC_EnumDevices(unsigned int, MV_CC_DEVICE_INFO_LIST* devices) {
    devices->nDeviceNum = 1;
    devices->pDeviceInfo[0] = &fake_device;
    return sdk.enum_result;
}

int MV_CC_CreateHandle(void** handle, const MV_CC_DEVICE_INFO*) {
    *handle = &fake_handle;
    return MV_OK;
}

int MV_CC_OpenDevice(void*, unsigned int, unsigned short) {
    return sdk.open_result;
}

int MV_CC_CloseDevice(void*) {
    return MV_OK;
}

int MV_CC_DestroyHandle(void* handle) {
    sdk.destroyed_handles.push_back(handle);
    return handle == &fake_handle ? MV_OK : MV_E_HANDLE;
}

int MV_CC_GetStringValue(void*, const char*, MVCC_STRINGVALUE* value) {
    std::copy_n("TESTCAM", 8, value->chCurValue);
    return sdk.serial_result;
}

int MV_CC_GetImageBuffer(void*, MV_FRAME_OUT* frame, unsigned int) {
    frame->stFrameInfo = sdk.frame_info;
    frame->pBufAddr = sdk.pixels.data();
    return sdk.grab_result;
}

int MV_CC_FreeImageBuffer(void*, MV_FRAME_OUT*) {
    ++sdk.freed_buffers;
    std::fill(sdk.pixels.begin(), sdk.pixels.end(), 0);
    return MV_OK;
}

int MV_CC_ConvertPixelType(void*, MV_CC_PIXEL_CONVERT_PARAM* param) {
    sdk.conversion_capacity = param->nDstBufferSize;
    const unsigned int expected_size = param->nWidth * param->nHeight * 3;
    EXPECT_EQ(param->nDstBufferSize, expected_size);
    if (sdk.convert_result != MV_OK) {
        return sdk.convert_result;
    }
    std::copy_n(param->pSrcData, expected_size, param->pDstBuffer);
    param->nDstLen = sdk.incomplete_conversion ? expected_size - 1 : expected_size;
    return MV_OK;
}

int MV_CC_StartGrabbing(void*) {
    if (sdk.on_start) {
        sdk.on_start();
    }
    return sdk.start_result;
}

int MV_CC_StopGrabbing(void*) {
    return MV_OK;
}

int MV_CC_SetBoolValue(void*, const char*, bool) {
    return MV_OK;
}

int MV_CC_SetFloatValue(void*, const char*, float) {
    return MV_OK;
}

int MV_CC_SetEnumValueByString(void*, const char*, const char*) {
    return MV_OK;
}

int MV_CC_SetBayerCvtQuality(void*, unsigned int) {
    return MV_OK;
}

} // extern "C"
