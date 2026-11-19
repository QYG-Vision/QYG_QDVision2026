#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "armor_detector/armor_detector_node.hpp"
#include "gtest/gtest.h"

namespace {

std::atomic<int> grab_calls {0};
MV_CC_DEVICE_INFO fake_device {};
int fake_handle;

TEST(CameraLifecycle, destroys_and_recreates_capture_node_without_shutting_down_ros) {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("use_state_machine_camera", true),
        rclcpp::Parameter("camera_6mm.enable_DeviceSerialNumber", false),
        rclcpp::Parameter("camera_8mm.enable_DeviceSerialNumber", false),
        rclcpp::Parameter("debug", false)
    });
    for (int iteration = 0; iteration < 2; ++iteration) {
        grab_calls = 0;
        auto node = std::make_shared<qd::auto_aim::ArmorDetectorNode>(options);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (grab_calls.load() == 0 && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        EXPECT_GT(grab_calls.load(), 0);
        EXPECT_THROW(std::make_shared<qd::auto_aim::ArmorDetectorNode>(options), std::runtime_error);
        const auto start = std::chrono::steady_clock::now();
        node.reset();
        EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(2));
        EXPECT_TRUE(rclcpp::ok());
    }
    rclcpp::shutdown();
}

} // namespace

// 模拟取帧超时，不访问真实相机；验证退出不依赖全局 shutdown。
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

int MV_CC_OpenDevice(void*, unsigned int, unsigned short) { return MV_OK; }
int MV_CC_CloseDevice(void*) { return MV_OK; }
int MV_CC_DestroyHandle(void*) { return MV_OK; }
int MV_CC_StartGrabbing(void*) { return MV_OK; }
int MV_CC_StopGrabbing(void*) { return MV_OK; }
int MV_CC_SetBoolValue(void*, const char*, bool) { return MV_OK; }
int MV_CC_SetFloatValue(void*, const char*, float) { return MV_OK; }
int MV_CC_SetEnumValueByString(void*, const char*, const char*) { return MV_OK; }
int MV_CC_SetBayerCvtQuality(void*, unsigned int) { return MV_OK; }

int MV_CC_GetStringValue(void*, const char*, MVCC_STRINGVALUE* value) {
    std::copy_n("TESTCAM", 8, value->chCurValue);
    return MV_OK;
}

int MV_CC_GetImageBuffer(void*, MV_FRAME_OUT*, unsigned int) {
    ++grab_calls;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return MV_E_HANDLE;
}

} // extern "C"
