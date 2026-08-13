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

    Yolov5Detector(
        const std::string& model_path_xml,
        const std::string& model_path_bin,
        const std::string& device,
        const EnemyColor& color,
        const Yolov5Params& y
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