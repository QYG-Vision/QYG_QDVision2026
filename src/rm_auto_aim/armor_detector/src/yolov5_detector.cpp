//
// Neural network detector for armor detection using yolov5.
// Created by cwb on 2025.11.8.
//

//opencv
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
//std
#include <string>
#include <vector>
//project
#include "armor_detector/types.hpp"
#include "armor_detector/yolov5_detector.hpp"
#include "rm_utils/logger/log.hpp"
#include <fmt/format.h>

namespace qd::auto_aim {

Yolov5Detector::Yolov5Detector(
    const std::string& model_path_xml,
    const std::string& model_path_bin,
    const std::string& device,
    const EnemyColor& color,
    const Yolov5Params& y
):
    device_(device),
    yolov5_params(y) {
    this->detect_color = color;
    input_shape = { 1,
                    static_cast<unsigned long>(IMAGE_HEIGHT),
                    static_cast<unsigned long>(IMAGE_WIDTH),
                    3 };
    model = core.read_model(model_path_xml, model_path_bin);
    // Step . Inizialize Preprocessing for the model
    ppp_ = std::make_unique<ov::preprocess::PrePostProcessor>(model);
    // Specify input image format
    ppp_->input()
        .tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::RGB);
    // NHWC:batchsize,height,width,channels
    // Specify preprocess pipeline to input image without resizing
    ppp_->input().preprocess().convert_element_type(ov::element::f32).scale({ 255., 255., 255. });
    //  Specify model's input layout
    ppp_->input().model().set_layout("NCHW");
    // Specify output results format
    ppp_->output().tensor().set_element_type(ov::element::f32);
    // Embed above steps in the graph
    model = ppp_->build();

    try {
        compiled_model = core.compile_model(model, device, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    } catch (const std::exception& e) {
        FYT_ERROR("armor_detector", "Model compilation to device {} failed: {}", device, e.what());
        throw;
    }

    infer_request_ = compiled_model.create_infer_request();

    input_buffer_ = cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    input_tensor_ = ov::Tensor(compiled_model.input().get_element_type(), 
                               compiled_model.input().get_shape(), 
                               input_buffer_.data);
}

std::vector<Armor> Yolov5Detector::detect(const cv::Mat& input) {
    lights_.clear();
    armors_.clear();
    debug_yolos.data.clear();
    // scale factor
    float scale = 1.0f;
    // get armors from yolov5 model
    armors_ = getYoloOutput(input, scale);
    // remove armors with ignore classes
    if (!armors_.empty()) {
        eraseIgnoreYoloClasses(armors_);
    }

    return armors_;
}

std::vector<Armor> Yolov5Detector::getYoloOutput(const cv::Mat& img, float& scale) {
    //get img size
    int w = img.cols;
    int h = img.rows;
    // calculate scale factor
    scale = std::min(640.0f / w, 640.0f / h);
    int nw = static_cast<int>(w * scale);
    int nh = static_cast<int>(h * scale);
    // prepare input buffer
    input_buffer_.setTo(cv::Scalar(0, 0, 0));
    // resize input image
    cv::Rect roi(0, 0, nw, nh);
    cv::resize(img, input_buffer_(roi), cv::Size(nw, nh));

    // start inference
    infer_request_.set_input_tensor(input_tensor_);

    double start_time = cv::getTickCount();
    infer_request_.infer();
    double end_time = cv::getTickCount();
    // record inference time
    double inference_time = (end_time - start_time) / cv::getTickFrequency() * 1000;
    FYT_DEBUG("armor_detector", "YOLOv5 inference time: {:.2f}ms", inference_time);

    // get output
    auto output = infer_request_.get_output_tensor(0);
    ov::Shape output_shape = output.get_shape();
    cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, output.data());

    return infer(output_buffer, scale);
}

std::vector<Armor> Yolov5Detector::infer(const cv::Mat& output, float scale) {
    std::vector<Armor> armors;
    // get conf threshold
    float conf_threshold = yolov5_params.conf_thresh;

    // parse output
    for (int i = 0; i < output.rows; i++) {
        // filter confidence
        float confidence = output.at<float>(i, 8);
        confidence = sigmoid(confidence);
        if (confidence < conf_threshold) {
            continue;
        }

        // get color and class scores
        cv::Mat color_scores = output.row(i).colRange(9, 13);
        cv::Mat classes_scores = output.row(i).colRange(13, 22);
        cv::Point class_id, color_id;
        double score_color, score_num;
        cv::minMaxLoc(classes_scores, nullptr, &score_num, nullptr, &class_id);
        cv::minMaxLoc(color_scores, nullptr, &score_color, nullptr, &color_id);

        // filter color
        int detect_color_int = (detect_color == EnemyColor::BLUE) ? 0 : 1;
        if (color_id.x != detect_color_int) {
            continue; // skip non-detect color
        }

        // get bounding box points and scale to original size
        cv::Point2f top_left(output.at<float>(i, 0) / scale, output.at<float>(i, 1) / scale);
        cv::Point2f top_right(output.at<float>(i, 6) / scale, output.at<float>(i, 7) / scale);
        cv::Point2f bottom_right(output.at<float>(i, 4) / scale, output.at<float>(i, 5) / scale);
        cv::Point2f bottom_left(output.at<float>(i, 2) / scale, output.at<float>(i, 3) / scale);

        // set armor color
        EnemyColor armor_color =
            (color_id.x == 0) ? EnemyColor::BLUE : EnemyColor::RED; // use detect color

        // create left and right light
        Light left_light = createLightBar(top_left, bottom_left, armor_color);
        Light right_light = createLightBar(top_right, bottom_right, armor_color);
        lights_.push_back(left_light);
        lights_.push_back(right_light);

        // create armor
        Armor armor(left_light, right_light);
        armor.type = getArmorType(left_light, right_light);
        armor.number = ArmorNumber[class_id.x];
        armor.confidence = confidence;
        armor.classfication_result =
            fmt::format("{}:{:.1f}%", armor.number, armor.confidence * 100.0);
        armors.emplace_back(armor);

        // Fill in debug information
        rm_interfaces::msg::DebugYolo yolo_data;
        yolo_data.confidence = confidence;
        yolo_data.color_id = color_id.x;
        yolo_data.class_id = class_id.x;
        this->debug_yolos.data.emplace_back(yolo_data);
    }

    applyNMS(armors);

    return armors;
}

Light Yolov5Detector::createLightBar(
    const cv::Point2f& top,
    const cv::Point2f& bottom,
    EnemyColor color
) {
    // Create light object
    Light light;

    // Set all necessary properties
    light.top = top;
    light.bottom = bottom;
    light.center = (top + bottom) / 2.0f;
    light.color = color;

    // Calculate basic geometry properties
    light.length = cv::norm(top - bottom);
    light.width = light.length * Light_Ratio;

    // Calculate direction vector
    light.axis = top - bottom;
    light.axis = light.axis / cv::norm(light.axis);

    // Calculate tilt angle
    light.tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    light.tilt_angle = light.tilt_angle / CV_PI * 180;

    return light;
}

ArmorType Yolov5Detector::getArmorType(const Light& light_1, const Light& light_2) {
    // Distance between the center of 2 lights (unit : light length)
    float avg_light_length = (light_1.length + light_2.length) / 2;
    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;

    // Judge armor type
    ArmorType type;
    if (center_distance > yolov5_params.min_large_center_distance) {
        type = ArmorType::LARGE;
    } else {
        type = ArmorType::SMALL;
    }

    return type;
}

void Yolov5Detector::applyNMS(std::vector<Armor>& armors) {
    if (armors.empty())
        return;

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;

    // create boxes and confidences
    for (const auto& armor: armors) {
        auto landmarks = armor.landmarks();
        float min_x = landmarks[0].x, max_x = landmarks[0].x;
        float min_y = landmarks[0].y, max_y = landmarks[0].y;

        for (const auto& point: landmarks) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }

        boxes.push_back(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y));
        confidences.push_back(armor.confidence);
    }

    // apply NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(
        boxes,
        confidences,
        yolov5_params.conf_thresh,
        yolov5_params.nms_thresh,
        indices
    );

    // filter armors
    std::vector<Armor> filtered_armors;
    for (int index: indices) {
        if (static_cast<size_t>(index) < armors.size()) {
            filtered_armors.push_back(armors[index]);
        }
    }

    armors.swap(filtered_armors);
}

void Yolov5Detector::eraseIgnoreYoloClasses(std::vector<Armor>& armors) {
    armors.erase(
        std::remove_if(
            armors.begin(),
            armors.end(),
            [this](const Armor& armor) {
                for (const auto& number: yolov5_params.ignore_yoloclasses) {
                    if (armor.number == number) {
                        return true;
                    }
                }

                bool mismatch_armor_type = false;
                if (armor.type == ArmorType::LARGE) {
                    mismatch_armor_type = armor.number == "outpost" || armor.number == "2"
                        || armor.number == "sentry" || armor.number == "base";
                } else if (armor.type == ArmorType::SMALL) {
                    mismatch_armor_type = armor.number == "1";
                }
                return mismatch_armor_type;
            }
        ),
        armors.end()
    );
}

double Yolov5Detector::sigmoid(double x) {
    if (x > 0)
        return 1.0 / (1.0 + exp(-x));
    else
        return exp(x) / (1.0 + exp(x));
}

void Yolov5Detector::drawResults(cv::Mat& img) const noexcept {
    // Draw Lights

    for (const auto& light: lights_) {
        auto line_color =
            light.color == EnemyColor::RED ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 255, 0);
        // cv::ellipse(img, light, line_color, 2);
        cv::line(img, light.top, light.bottom, line_color, 1);
    }

    // Draw armors
    for (const auto& armor: armors_) {
        cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 1);
        cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1);

        // cv::line(
        //   img, armor.left_light.top, armor.left_light.bottom, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        // cv::line(
        //   img, armor.right_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        // cv::line(
        //   img, armor.left_light.top, armor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        // cv::line(img,
        //          armor.right_light.bottom,
        //          armor.left_light.bottom,
        //          cv::Scalar(0, 255, 0),
        //          1,
        //          cv::LINE_AA);
    }
    // Show numbers and confidence
    // for (const auto &armor : armors_) {
    //   std::string text =
    //     fmt::format("{} {}", armorTypeToString(armor.type), armor.classfication_result);
    //   cv::putText(
    //     img, text, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
    // }
}

} // namespace qd::auto_aim
