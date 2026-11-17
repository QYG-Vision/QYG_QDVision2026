//
// Neural network detector for armor detection using yolov5.
// Created by cwb on 2025.11.8.
//

#ifndef ARMOR_DETECTOR_YOLOV5_DETECTOR_HPP_
#define ARMOR_DETECTOR_YOLOV5_DETECTOR_HPP_

// openvino
#include <openvino/openvino.hpp>
// std
#include <string>
#include <vector>
// project
#include "armor_detector/detector.hpp"
#include "armor_detector/tensorrt_inference.hpp"
#include "armor_detector/types.hpp"
#include "rm_interfaces/msg/debug_yolos.hpp"

namespace qd::auto_aim {
class Yolov5Detector: public Detector {
public:
    struct Yolov5Params {
        // confidence threshold
        double conf_thresh;
        // nms threshold
        double nms_thresh;
        // Armor params
        double min_large_center_distance;
        // ingore yolo classes
        std::vector<std::string> ignore_yoloclasses;
    };

    /**
     * @brief 根据选定后端加载装甲板模型，共用角点和类别解析逻辑。
     * @param model_path_xml OpenVINO XML 路径或 TensorRT engine 路径。
     * @param model_path_bin OpenVINO 权重路径，TensorRT 模式忽略。
     * @param device OpenVINO 推理设备，TensorRT 模式使用当前 CUDA 设备。
     * @param color 需要识别的敌方颜色。
     * @param y 置信度、NMS 及装甲板分类参数。
     * @param backend 推理后端，只允许 openvino 或 tensorrt。
     */
    Yolov5Detector(
        const std::string& model_path_xml,
        const std::string& model_path_bin,
        const std::string& device,
        const EnemyColor& color,
        const Yolov5Params& y,
        const std::string& backend = "openvino"
    );

    // YOLOv5 detect armor
    std::vector<Armor> detect(const cv::Mat& input) override;

    // For debug usage
    void drawResults(cv::Mat& img) const noexcept override;

    // Debug msgs
    rm_interfaces::msg::DebugYolos debug_yolos;

    // Parameters
    std::string device_;
    Yolov5Params yolov5_params;

private:
    const int IMAGE_HEIGHT = 640;
    const int IMAGE_WIDTH = 640;
    static constexpr double Light_Ratio = 0.6 / 5.5;
    std::shared_ptr<ov::Model> model;
    ov::Core core;
    std::unique_ptr<ov::preprocess::PrePostProcessor> ppp_;
    ov::CompiledModel compiled_model;
    ov::Shape input_shape;
    ov::InferRequest infer_request_;
    cv::Mat input_buffer_;
    ov::Tensor input_tensor_;
    std::unique_ptr<TensorRtInference> tensorrt_;

    const std::vector<std::string> ArmorNumber = {
        "sentry", "1", "2", "3", "4", "5", "outpost", "base", "base", "negative"
    };

    std::vector<Armor> getYoloOutput(const cv::Mat& img, float& scale);
    std::vector<Armor> infer(const cv::Mat& output, float scale);
    Light createLightBar(const cv::Point2f& top, const cv::Point2f& bottom, EnemyColor color);
    ArmorType getArmorType(const Light& light_1, const Light& light_2);
    void applyNMS(std::vector<Armor>& armors);
    void eraseIgnoreYoloClasses(std::vector<Armor>& armors);
    double sigmoid(double x);
};

} // namespace qd::auto_aim

#endif //ARMOR_DETECTOR_YOLOV5_DETECTOR_HPP_
