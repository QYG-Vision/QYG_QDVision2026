//
// detector parent class
// created by cwb on 2026.1.22
//

#ifndef ARMOR_DETECTOR_DETECTOR_HPP_
#define ARMOR_DETECTOR_DETECTOR_HPP_

// project
#include "armor_detector/types.hpp"
// std
#include <vector>
// OpenCV
#include <opencv2/core.hpp>

namespace qd::auto_aim {
class Detector {
public:
    virtual ~Detector() = default;

    virtual std::vector<Armor> detect(const cv::Mat& input) = 0;
    virtual void drawResults(cv::Mat& img) const noexcept = 0;

    EnemyColor detect_color;

protected:
    std::vector<Light> lights_;
    std::vector<Armor> armors_;
};

} // namespace qd::auto_aim

#endif // ARMOR_DETECTOR_DETECTOR_HPP_