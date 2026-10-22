#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "rm_utils/driver/HikCamera.hpp"
#include "rm_utils/vision_state_machine.hpp"
#include "gtest/gtest.h"

namespace {

struct FakeSdkState {
    int open_result { MV_OK };
    int start_result { MV_OK };
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
}

} // namespace

// 使用与 SDK 头文件相同的 C 接口替身，检查实际传入的句柄并隔离真实硬件。
extern "C" {

int MV_CC_EnumDevices(unsigned int, MV_CC_DEVICE_INFO_LIST* devices) {
    devices->nDeviceNum = 1;
    devices->pDeviceInfo[0] = &fake_device;
    return MV_OK;
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
