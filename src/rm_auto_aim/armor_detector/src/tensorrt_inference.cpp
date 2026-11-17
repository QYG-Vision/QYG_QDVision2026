#include "armor_detector/tensorrt_inference.hpp"

#include <stdexcept>

#ifdef QD_HAS_TENSORRT
    #include <NvInfer.h>
    #include <NvInferVersion.h>
    #include <cstdio>
    #include <cuda_runtime_api.h>
    #include <fstream>
    #include <iterator>
    #include <opencv2/dnn.hpp>
    #include <vector>

    #if NV_TENSORRT_MAJOR != 10
        #error "The Jetson inference backend requires TensorRT 10.x"
    #endif

namespace qd::auto_aim {
namespace {

    class TensorRtLogger: public nvinfer1::ILogger {
    public:
        /**
         * @brief 输出 TensorRT 错误，使用线程安全的标准错误流。
         * @param severity 日志严重程度。
         * @param message TensorRT 提供的日志内容，所有权属于调用方。
         */
        void log(Severity severity, const char* message) noexcept override {
            if (severity <= Severity::kERROR) {
                std::fprintf(stderr, "[TensorRT] %s\n", message);
            }
        }
    };

    /**
     * @brief 将 CUDA 错误转为包含操作名称的异常。
     * @param status CUDA API 返回状态。
     * @param operation 失败时显示的操作名称。
     */
    void check_cuda(cudaError_t status, const char* operation) {
        if (status != cudaSuccess) {
            throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
        }
    }

} // namespace

class TensorRtInference::Impl {
public:
    ~Impl() {
        // 构造中途失败时也由外层 unique_ptr 回收已经创建的资源。
        if (stream != nullptr || input_device != nullptr || output_device != nullptr) {
            int previous_device = 0;
            const bool restore = cudaGetDevice(&previous_device) == cudaSuccess;
            cudaSetDevice(device);
            if (stream != nullptr) {
                cudaStreamSynchronize(stream);
            }
            context.reset();
            engine.reset();
            runtime.reset();
            if (input_device != nullptr) {
                cudaFree(input_device);
            }
            if (output_device != nullptr) {
                cudaFree(output_device);
            }
            if (stream != nullptr) {
                cudaStreamDestroy(stream);
            }
            if (restore) {
                cudaSetDevice(previous_device);
            }
        }
    }

    /**
     * @brief 检查 engine 的张量契约并初始化 CUDA 缓冲区。
     * @param engine_path TensorRT engine 文件路径。
     */
    void initialize(const std::string& engine_path) {
        check_cuda(cudaGetDevice(&device), "cudaGetDevice");
        std::ifstream file(engine_path, std::ios::binary);
        if (!file) {
            throw std::runtime_error("Cannot open TensorRT engine: " + engine_path);
        }
        const std::vector<char> serialized(
            (std::istreambuf_iterator<char>(file)),
            std::istreambuf_iterator<char>()
        );
        if (serialized.empty()) {
            throw std::runtime_error("TensorRT engine is empty: " + engine_path);
        }
        runtime.reset(nvinfer1::createInferRuntime(logger));
        if (runtime == nullptr) {
            throw std::runtime_error("Cannot create TensorRT runtime; see TensorRT log");
        }
        engine.reset(runtime->deserializeCudaEngine(serialized.data(), serialized.size()));
        if (engine == nullptr) {
            throw std::runtime_error(
                "Cannot deserialize engine; rebuild it on this Jetson with TensorRT 10.3"
            );
        }
        if (engine->getNbIOTensors() != 2) {
            throw std::runtime_error("Armor engine must have exactly one input and one output");
        }
        for (int index = 0; index < engine->getNbIOTensors(); ++index) {
            const char* name = engine->getIOTensorName(index);
            if (engine->getTensorDataType(name) != nvinfer1::DataType::kFLOAT
                || engine->getTensorFormat(name) != nvinfer1::TensorFormat::kLINEAR
                || engine->getTensorLocation(name) != nvinfer1::TensorLocation::kDEVICE)
            {
                throw std::runtime_error("Engine I/O must use device FP32 LINEAR tensors");
            }
            if (engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
                input_name = name;
            } else {
                output_name = name;
            }
        }
        if (input_name.empty() || output_name.empty()) {
            throw std::runtime_error("Armor engine must have one input and one output");
        }
        const auto input_shape = engine->getTensorShape(input_name.c_str());
        const auto output_shape = engine->getTensorShape(output_name.c_str());
        if (input_shape.nbDims != 4 || input_shape.d[0] != 1 || input_shape.d[1] != 3
            || input_shape.d[2] != 640 || input_shape.d[3] != 640 || output_shape.nbDims != 3
            || output_shape.d[0] != 1 || output_shape.d[1] <= 0 || output_shape.d[1] > 1000000
            || output_shape.d[2] != 22)
        {
            throw std::runtime_error(
                "Armor engine requires static [1,3,640,640] -> [1,N,22]; "
                "use the original four-corner armor model, without embedded preprocessing/NMS"
            );
        }
        output.create(static_cast<int>(output_shape.d[1]), 22, CV_32F);
        context.reset(engine->createExecutionContext());
        if (context == nullptr) {
            throw std::runtime_error("Cannot create TensorRT execution context; see TensorRT log");
        }
        check_cuda(cudaStreamCreate(&stream), "cudaStreamCreate");
        check_cuda(cudaMalloc(&input_device, INPUT_BYTES), "cudaMalloc input");
        check_cuda(cudaMalloc(&output_device, output.total() * sizeof(float)), "cudaMalloc output");
        if (!context->setTensorAddress(input_name.c_str(), input_device)
            || !context->setTensorAddress(output_name.c_str(), output_device))
        {
            throw std::runtime_error("Cannot bind TensorRT I/O buffers; see TensorRT log");
        }
    }

    TensorRtLogger logger;
    std::unique_ptr<nvinfer1::IRuntime> runtime;
    std::unique_ptr<nvinfer1::ICudaEngine> engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    std::string input_name;
    std::string output_name;
    int device = 0;
    cudaStream_t stream = nullptr;
    void* input_device = nullptr;
    void* output_device = nullptr;
    cv::Mat input;
    cv::Mat output;
    static constexpr size_t INPUT_BYTES = 3 * 640 * 640 * sizeof(float);
};

TensorRtInference::TensorRtInference(const std::string& engine_path):
    impl_(std::make_unique<Impl>()) {
    impl_->initialize(engine_path);
}

TensorRtInference::~TensorRtInference() = default;

cv::Mat TensorRtInference::infer(const cv::Mat& rgb_image) {
    if (rgb_image.type() != CV_8UC3 || rgb_image.rows != 640 || rgb_image.cols != 640) {
        throw std::invalid_argument("TensorRT input must be a 640x640 CV_8UC3 RGB image");
    }
    check_cuda(cudaSetDevice(impl_->device), "cudaSetDevice");
    // 保持 OpenVINO 路径的 RGB、左上角对齐补黑边和除以 255 的处理约定。
    cv::dnn::blobFromImage(
        rgb_image,
        impl_->input,
        1.0 / 255.0,
        cv::Size(),
        cv::Scalar(),
        false,
        false,
        CV_32F
    );
    check_cuda(
        cudaMemcpyAsync(
            impl_->input_device,
            impl_->input.data,
            Impl::INPUT_BYTES,
            cudaMemcpyHostToDevice,
            impl_->stream
        ),
        "cudaMemcpyAsync input"
    );
    if (!impl_->context->enqueueV3(impl_->stream)) {
        throw std::runtime_error("TensorRT enqueueV3 failed; see TensorRT log");
    }
    check_cuda(
        cudaMemcpyAsync(
            impl_->output.data,
            impl_->output_device,
            impl_->output.total() * sizeof(float),
            cudaMemcpyDeviceToHost,
            impl_->stream
        ),
        "cudaMemcpyAsync output"
    );
    check_cuda(cudaStreamSynchronize(impl_->stream), "cudaStreamSynchronize");
    return impl_->output;
}

} // namespace qd::auto_aim

#else

namespace qd::auto_aim {

class TensorRtInference::Impl {};

TensorRtInference::TensorRtInference(const std::string&) {
    throw std::runtime_error(
        "TensorRT backend is disabled; rebuild armor_detector with -DENABLE_TENSORRT=ON "
        "inside the Jetson CUDA/TensorRT development container"
    );
}

TensorRtInference::~TensorRtInference() = default;

cv::Mat TensorRtInference::infer(const cv::Mat&) {
    throw std::runtime_error("TensorRT backend is disabled");
}

} // namespace qd::auto_aim

#endif
