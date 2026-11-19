#include "armor_detector/tensorrt_inference.hpp"

#include <stdexcept>

#ifdef QD_HAS_TENSORRT
    #include <NvInfer.h>
    #include <NvInferVersion.h>
    #include <cstdio>
    #include <cuda_runtime_api.h>
    #include <fstream>
    #include <iterator>
    #include <vector>

    #include "armor_detector/cuda_preprocess.hpp"

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
        if (stream != nullptr || input_device != nullptr || output_device != nullptr
            || rgb_device != nullptr || rgb_host != nullptr || output_host != nullptr)
        {
            int previous_device = 0;
            const bool restore = cudaGetDevice(&previous_device) == cudaSuccess;
            cudaSetDevice(device);
            if (stream != nullptr) {
                cudaStreamSynchronize(stream);
            }
            if (graph_exec != nullptr) {
                cudaGraphExecDestroy(graph_exec);
            }
            if (graph != nullptr) {
                cudaGraphDestroy(graph);
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
            if (rgb_device != nullptr) {
                cudaFree(rgb_device);
            }
            if (rgb_host != nullptr) {
                cudaFreeHost(rgb_host);
            }
            if (output_host != nullptr) {
                cudaFreeHost(output_host);
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
        const auto output_rows = static_cast<int>(output_shape.d[1]);
        context.reset(engine->createExecutionContext());
        if (context == nullptr) {
            throw std::runtime_error("Cannot create TensorRT execution context; see TensorRT log");
        }
        check_cuda(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), "cudaStreamCreate");
        check_cuda(cudaMallocHost(&rgb_host, RGB_BYTES), "cudaMallocHost RGB");
        check_cuda(
            cudaMallocHost(&output_host, static_cast<std::size_t>(output_rows) * 22 * sizeof(float)),
            "cudaMallocHost output"
        );
        input = cv::Mat(640, 640, CV_8UC3, rgb_host);
        output = cv::Mat(output_rows, 22, CV_32F, output_host);
        check_cuda(cudaMalloc(&rgb_device, RGB_BYTES), "cudaMalloc RGB");
        check_cuda(cudaMalloc(&input_device, INPUT_BYTES), "cudaMalloc input");
        check_cuda(cudaMalloc(&output_device, output.total() * sizeof(float)), "cudaMalloc output");
        if (!context->setTensorAddress(input_name.c_str(), input_device)
            || !context->setTensorAddress(output_name.c_str(), output_device))
        {
            throw std::runtime_error("Cannot bind TensorRT I/O buffers; see TensorRT log");
        }
    }

    /** @brief 在同一非阻塞流上提交传输、预处理、模型和结果回传。 */
    void enqueue() {
        check_cuda(cudaMemcpyAsync(rgb_device, rgb_host, RGB_BYTES,
                                   cudaMemcpyHostToDevice, stream), "cudaMemcpyAsync input");
        check_cuda(normalizeRgbCuda(static_cast<const std::uint8_t*>(rgb_device),
                                    static_cast<float*>(input_device), 640 * 640, stream),
                   "normalizeRgbCuda");
        if (!context->enqueueV3(stream)) {
            throw std::runtime_error("TensorRT enqueueV3 failed; see TensorRT log");
        }
        check_cuda(cudaMemcpyAsync(output_host, output_device, output.total() * sizeof(float),
                                   cudaMemcpyDeviceToHost, stream), "cudaMemcpyAsync output");
    }

    /**
     * @brief 正常推理预热后捕获固定地址的执行图，不兼容时保留普通提交路径。
     * @note 仅尝试一次；捕获期间不改变上下文、形状或任何缓冲区地址。
     */
    void capture_graph() {
        capture_attempted = true;
        auto status = cudaStreamBeginCapture(stream, cudaStreamCaptureModeThreadLocal);
        if (status != cudaSuccess) {
            std::fprintf(stderr, "[TensorRT] CUDA graph capture unavailable: %s\n",
                         cudaGetErrorString(status));
            cudaGetLastError();
            return;
        }
        bool enqueued = true;
        try {
            enqueue();
        } catch (const std::exception& error) {
            enqueued = false;
            std::fprintf(stderr, "[TensorRT] CUDA graph enqueue failed: %s\n", error.what());
        }
        // 即使提交失败也必须结束捕获，解除流的捕获状态。
        status = cudaStreamEndCapture(stream, &graph);
        if (status == cudaSuccess && enqueued && graph != nullptr) {
            status = cudaGraphInstantiate(&graph_exec, graph, 0);
        }
        if (status != cudaSuccess || !enqueued || graph_exec == nullptr) {
            if (graph_exec != nullptr) {
                cudaGraphExecDestroy(graph_exec);
                graph_exec = nullptr;
            }
            if (graph != nullptr) {
                cudaGraphDestroy(graph);
                graph = nullptr;
            }
            cudaGetLastError();
            std::fprintf(stderr, "[TensorRT] CUDA graph unavailable; using regular enqueue\n");
            return;
        }
        std::fprintf(stderr, "[TensorRT] CUDA graph enabled (transfer + preprocess + inference)\n");
    }

    TensorRtLogger logger;
    std::unique_ptr<nvinfer1::IRuntime> runtime;
    std::unique_ptr<nvinfer1::ICudaEngine> engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    std::string input_name;
    std::string output_name;
    int device = 0;
    cudaStream_t stream = nullptr;
    cudaGraph_t graph = nullptr;
    cudaGraphExec_t graph_exec = nullptr;
    bool use_cuda_graph = true;
    bool capture_attempted = false;
    void* input_device = nullptr;
    void* output_device = nullptr;
    void* rgb_device = nullptr;
    void* rgb_host = nullptr;
    void* output_host = nullptr;
    cv::Mat input;
    cv::Mat output;
    static constexpr size_t INPUT_BYTES = 3 * 640 * 640 * sizeof(float);
    static constexpr size_t RGB_BYTES = 3 * 640 * 640;
};

TensorRtInference::TensorRtInference(const std::string& engine_path, bool use_cuda_graph):
    impl_(std::make_unique<Impl>()) {
    impl_->use_cuda_graph = use_cuda_graph;
    impl_->initialize(engine_path);
}

TensorRtInference::~TensorRtInference() = default;

cv::Mat TensorRtInference::infer(const cv::Mat& rgb_image) {
    if (rgb_image.type() != CV_8UC3 || rgb_image.rows != 640 || rgb_image.cols != 640) {
        throw std::invalid_argument("TensorRT input must be a 640x640 CV_8UC3 RGB image");
    }
    check_cuda(cudaSetDevice(impl_->device), "cudaSetDevice");
    // 固定页内存支持真正异步传输；保留原 RGB 和缩放补边规则。
    rgb_image.copyTo(impl_->input);
    if (impl_->graph_exec != nullptr) {
        check_cuda(cudaGraphLaunch(impl_->graph_exec, impl_->stream), "cudaGraphLaunch");
    } else {
        impl_->enqueue();
    }
    check_cuda(cudaStreamSynchronize(impl_->stream), "cudaStreamSynchronize");
    if (impl_->use_cuda_graph && !impl_->capture_attempted) {
        // 首帧正常执行以完成 TensorRT 延迟初始化，再捕获后续帧共用的执行序列。
        impl_->capture_graph();
    }
    return impl_->output;
}

bool TensorRtInference::usesCudaGraph() const noexcept {
    return impl_->graph_exec != nullptr;
}

} // namespace qd::auto_aim

#else

namespace qd::auto_aim {

class TensorRtInference::Impl {};

TensorRtInference::TensorRtInference(const std::string&, bool) {
    throw std::runtime_error(
        "TensorRT backend is disabled; rebuild armor_detector with -DENABLE_TENSORRT=ON "
        "inside the Jetson CUDA/TensorRT development container"
    );
}

TensorRtInference::~TensorRtInference() = default;

cv::Mat TensorRtInference::infer(const cv::Mat&) {
    throw std::runtime_error("TensorRT backend is disabled");
}

bool TensorRtInference::usesCudaGraph() const noexcept {
    return false;
}

} // namespace qd::auto_aim

#endif
