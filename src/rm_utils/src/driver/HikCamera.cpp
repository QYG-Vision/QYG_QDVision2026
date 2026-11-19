#include "rm_utils/driver/HikCamera.hpp"

#include <limits>
#include <memory>

namespace qd::Device {

HikCamera::HikCamera(const Parameters& params): queue_(1), camera_handle_(nullptr), running_(true) {
    FYT_REGISTER_LOGGER("hik_camera", "qd2026-log", INFO);
    this->params_ = params;

    // 创建句柄
    this->camera_handle_ = get_camera_handle(params_);
}

HikCamera::~HikCamera() {
    running_ = false;
    // 等待取图线程退出后再释放句柄，避免线程继续访问已关闭的设备。
    if (daemon_thread_.joinable()) {
        daemon_thread_.join();
    }
    if (camera_handle_ != nullptr) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        // DestroyHandle 接收句柄本身，只有 CreateHandle 才接收句柄地址。
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
}

void HikCamera::starThread() {
    this->daemon_thread_ = std::thread(&HikCamera::image_callback, this);
}

void HikCamera::image_callback() {
    while (running_) {
        if (this->camera_handle_ == nullptr) {
            std::cout << " camera_handle_ is null!!!" << std::endl;
            return;
        }
        
        cv::Mat img;
        std::chrono::system_clock::time_point timestamp;
        get_img(img, timestamp);
        if (!img.empty()) {
            queue_.push({ img, timestamp });
        }
    }
}

void HikCamera::get_img(cv::Mat& img, std::chrono::system_clock::time_point& timestamp) {
    img.release();
    timestamp = {};
    if (this->camera_handle_ == nullptr) {
        std::cout << " camera_handle_ is null!!!" << std::endl;
        return;
    }

    // 获得图像缓存
    MV_FRAME_OUT frame {};
    const int grab_result = MV_CC_GetImageBuffer(camera_handle_, &frame, 1000);
    if (grab_result != MV_OK) {
        printf("No image data! nRet [0x%x]\n", grab_result);
        return;
    }

    // SDK 缓冲区必须在所有失败路径和 OpenCV 异常路径上归还。
    auto release_frame = [this](MV_FRAME_OUT* buffer) {
        MV_CC_FreeImageBuffer(camera_handle_, buffer);
    };
    std::unique_ptr<MV_FRAME_OUT, decltype(release_frame)> frame_guard(&frame, release_frame);
    const auto& info = frame.stFrameInfo;
    const size_t pixel_count = static_cast<size_t>(info.nWidth) * info.nHeight;
    if (!frame.pBufAddr || pixel_count == 0 || info.nFrameLen == 0
        || pixel_count > std::numeric_limits<unsigned int>::max() / 3)
    {
        FYT_WARN("hik_camera", "Invalid image buffer or dimensions");
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    cv::Mat dst_img;
    try {
        // OpenCV 短 Bayer 名称与传感器排列相反；以下映射输出 RGB，兼容旧版 OpenCV。
        const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
            { PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR },
            { PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR },
            { PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR },
            { PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR }
        };
        const auto conversion = type_map.find(info.enPixelType);
        if (params_.enable_opencv && conversion != type_map.end()) {
            if (info.nFrameLen < pixel_count) {
                FYT_WARN("hik_camera", "Truncated Bayer image buffer");
                return;
            }
            cv::Mat raw(info.nHeight, info.nWidth, CV_8UC1, frame.pBufAddr);
            cv::cvtColor(raw, dst_img, conversion->second);
        } else {
            dst_img.create(info.nHeight, info.nWidth, CV_8UC3);
            MV_CC_PIXEL_CONVERT_PARAM convert_param {};
            convert_param.nWidth = info.nWidth;
            convert_param.nHeight = info.nHeight;
            convert_param.enSrcPixelType = info.enPixelType;
            convert_param.pSrcData = frame.pBufAddr;
            convert_param.nSrcDataLen = info.nFrameLen;
            convert_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            convert_param.pDstBuffer = dst_img.data;
            convert_param.nDstBufferSize =
                static_cast<unsigned int>(dst_img.total() * dst_img.elemSize());
            const int convert_result = MV_CC_ConvertPixelType(camera_handle_, &convert_param);
            if (convert_result != MV_OK || convert_param.nDstLen != convert_param.nDstBufferSize) {
                FYT_WARN("hik_camera", "Pixel conversion failed or returned incomplete RGB data: {:#x}",
                         convert_result);
                return;
            }
        }
    } catch (const cv::Exception& e) {
        FYT_WARN("hik_camera", "Pixel conversion failed: {}", e.what());
        return;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    FYT_DEBUG("hik_camera", "Latecy(ConvertPixel) : {:.2f} ms", elapsed.count());
    
    img = dst_img;
    timestamp = get_time(info.nHostTimeStamp);
}

cv::Mat HikCamera::get_cv_img() {
    return this->queue_.pop().img;
}

void HikCamera::read_img(cv::Mat& img, std::chrono::system_clock::time_point& timestamp) {
    CameraData data = this->queue_.pop();
    img = data.img;
    timestamp = data.timestamp;
}

void* HikCamera::get_camera_handle(const Parameters& params) {
    void* handle = nullptr;
    MVCC_STRINGVALUE string_value {};

    // 枚举设备
    MV_CC_DEVICE_INFO_LIST device_list {};
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (nRet != MV_OK) {
        printf("EnumDevices failed! nRet [0x%x]\n", nRet);
        return nullptr;
    }

    // 等待设备连接
    int count = 4;
    while (device_list.nDeviceNum == 0) {
        printf("No camera found! nRet [0x%x]\n", nRet);
        printf("waiting for camera to be connected...\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (count-- < 0) {
            return nullptr;
        }

        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
        if (nRet != MV_OK) {
            return nullptr;
        }
    }

    // 查找序列号
    MV_CC_DEVICE_INFO* device_info = device_list.pDeviceInfo[0];
    bool matched = false;
    for (unsigned int i = 0; i < device_list.nDeviceNum; i++) {
        if (!params.enable_DeviceSerialNumber) {
            break;
        }
        auto *device = device_list.pDeviceInfo[i];
        
        // 打开设备
        nRet = MV_CC_CreateHandle(&handle, device);
        if (nRet != MV_OK) {
            printf("CreateHandle failed! nRet [0x%x]\n", nRet);
            continue;
        }
        nRet = MV_CC_OpenDevice(handle);
        if (nRet != MV_OK) {
            printf("OpenDevice failed! nRet [0x%x]\n", nRet);
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            continue;
        }

        // 获取序列号
        string_value = {};
        const int serial_result = MV_CC_GetStringValue(handle, "DeviceSerialNumber", &string_value);
        std::cout << "Found DeviceSerialNumber: " << string_value.chCurValue << std::endl;

        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);

        // 比较序列号
        if (serial_result == MV_OK
            && std::string(string_value.chCurValue) == params.device_serial_number) {
            device_info = device;
            std::cout << "Found Target DeviceSerialNumber: " << string_value.chCurValue
                      << std::endl;
            matched = true;
            break;
        }
    }

    if (params.enable_DeviceSerialNumber && !matched) {
        std::cout << "enable_DeviceSerialNumber but matched is false" << std::endl;
        return nullptr;
    }

    // 打开设备
    nRet = MV_CC_CreateHandle(&handle, device_info);
    printf("MV_CC_CreateHandle! nRet [0x%x]\n", nRet);
    if (nRet != MV_OK) {
        printf("CreateHandle failed for target camera! nRet [0x%x]\n", nRet);
        return nullptr;
    }

    nRet = MV_CC_OpenDevice(handle);
    printf("MV_CC_OpenDevice! nRet [0x%x]\n", nRet);
    if (nRet != MV_OK) {
        printf("OpenDevice failed for target camera! nRet [0x%x]\n", nRet);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return nullptr;
    }

    MV_CC_GetStringValue(handle, "DeviceSerialNumber", &string_value);
    std::cout << "Select DeviceSerialNumber: " << string_value.chCurValue << std::endl;
    // 设置相机参数
    set_hik_param(handle, params);
    // 开始取流
    nRet = MV_CC_StartGrabbing(handle);
    printf("MV_CC_StartGrabbing! nRet [0x%x]\n", nRet);
    if (nRet != MV_OK) {
        printf("StartGrabbing failed! nRet [0x%x]\n", nRet);
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return nullptr;
    }

    return handle;
}

void HikCamera::set_hik_param(void* handle, const Parameters& params) {
    if (handle == nullptr) {
        std::cout << " handle is null!!!" << std::endl;
        return;
    }

    // 锁帧
    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", params.enable_frame_rate);
    printf("Set AcquisitionFrameRateEnable! nRet [0x%x]\n", nRet);
    // 帧率限制
    nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", params.frame_rate);
    printf("Set AcquisitionFrameRate! nRet [0x%x]\n", nRet);
    // 曝光
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", params.exposure_time);
    printf("Set ExposureTime! nRet [0x%x]\n", nRet);
    // 增益
    nRet = MV_CC_SetFloatValue(handle, "Gain", params.gain);
    printf("Set Gain! nRet [0x%x]\n", nRet);
    // 像素格式
    nRet = MV_CC_SetEnumValueByString(handle, "PixelFormat", params.pixel_format.c_str());
    printf("Set PixelFormat! nRet [0x%x]\n", nRet);
    // ADC比特深度
    nRet = MV_CC_SetEnumValueByString(handle, "ADCBitDepth", params.adc_bit_depth.c_str());
    printf("Set ADCBitDepth! nRet [0x%x]\n", nRet);
    //设置插值方法
    nRet = MV_CC_SetBayerCvtQuality(handle, params.cvt_quality);
    printf("Set BayerCvtQuality! nRet [0x%x]\n", nRet);
    // 画面水平、垂直翻转
    nRet = MV_CC_SetBoolValue(handle, "ReverseX", params.reverseXY);
    printf("Set ReverseX! nRet [0x%x]\n", nRet);
    nRet = MV_CC_SetBoolValue(handle, "ReverseY", params.reverseXY);
    printf("Set ReverseY! nRet [0x%x]\n", nRet);

    this->half_exposure_duration = std::chrono::duration_cast<std::chrono::system_clock::duration>(
        std::chrono::duration<double, std::micro>(params.exposure_time / 2.0)
    );
}

void HikCamera::update_camera_setting() {
    set_hik_param(this->camera_handle_, this->params_);
}

std::chrono::system_clock::time_point HikCamera::get_time() {
    return std::chrono::system_clock::now() - this->half_exposure_duration.load();
}

std::chrono::system_clock::time_point HikCamera::get_time(const int64_t& nHostTimeStamp) {
    // 1. 将 int64_t 的 ms 转换为毫秒 duration
    auto host_ts_duration = std::chrono::milliseconds(nHostTimeStamp);

    // 2. 曝光由参数回调线程更新，这里只读取一次快照。
    auto result_duration = host_ts_duration - this->half_exposure_duration.load();

    // 4. 将结果转换回系统时钟的 time_point
    // 注意：system_clock::time_point 通常是整数，需要进行 duration_cast
    std::chrono::system_clock::time_point img_time_center(
        std::chrono::duration_cast<std::chrono::system_clock::duration>(result_duration)
    );

    return img_time_center;
}

} // namespace qd::Device
