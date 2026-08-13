// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARMOR_SOLVER_MOTION_MODEL_HPP_
#define ARMOR_SOLVER_MOTION_MODEL_HPP_

// ceres
#include <ceres/ceres.h>
// project
#include "rm_utils/math/extended_kalman_filter.hpp"

namespace qd::auto_aim {

// State: xc(0), v_xc(1), yc(2), v_yc(3), zc(4), v_zc(5),
//        yaw(6), v_yaw(7), r(8), dr(9), dz1(10), dz2(11)
// dr: radius offset (odd armors use r+dr in 4-armor, armor 1 uses r+dr in 2-armor)
// dz1: z offset of armor 1 relative to zc (used in 3-armor and 2-armor)
// dz2: z offset of armor 2 relative to zc (used in 3-armor)
constexpr int X_N = 12, Z_N = 4;
constexpr int X_N_ = 6, Z_N_ = 3;

struct Predict {
    explicit Predict(double dt): dt(dt) {}

    template<typename T>
    void operator()(const T x0[X_N], T x1[X_N]) {
        for (int i = 0; i < X_N; i++) {
            x1[i] = x0[i];
        }
        x1[0] += x0[1] * dt;
        x1[2] += x0[3] * dt;
        x1[4] += x0[5] * dt;
        x1[6] += x0[7] * dt;
    }

    double dt;
};

struct Measure {
    int armor_offset;
    int armors_num;

    explicit Measure(int offset = 0, int num = 4): armor_offset(offset), armors_num(num) {}

    template<typename T>
    void operator()(const T x[X_N], T z[Z_N]) {
        T armor_yaw = x[6] + T(armor_offset * 2.0 * M_PI / armors_num);
        armor_yaw = ceres::atan2(ceres::sin(armor_yaw), ceres::cos(armor_yaw));

        T za = x[4];
        if (armors_num == 3) {
            if (armor_offset == 1)
                za = za + x[10];
            else if (armor_offset == 2)
                za = za + x[11];
        }

        T r = x[8];
        if (armors_num == 4 && (armor_offset == 1 || armor_offset == 3)) {
            r = r + x[9];
            za = za + x[10];
        }

        T xa = x[0] - ceres::cos(armor_yaw) * r;
        T ya = x[2] - ceres::sin(armor_yaw) * r;

        z[0] = ceres::atan2(ya, xa); // yaw
        z[1] = ceres::atan2(za, ceres::sqrt(xa * xa + ya * ya)); // pitch
        z[2] = ceres::sqrt(xa * xa + ya * ya + za * za); // distance
        z[3] = armor_yaw; // angle
    }
};

using RobotStateEKF = ExtendedKalmanFilter<X_N, Z_N, Predict, Measure>;

//平移运动模型
class EkfPredict {
public:
    explicit EkfPredict(const double& delta_t): delta_t(delta_t) {}
    template<typename T>
    void operator()(const T x_pre[X_N_], T x_cur[X_N_]) const {
        x_cur[0] = x_pre[0] + this->delta_t * x_pre[1];
        x_cur[1] = x_pre[1];
        x_cur[2] = x_pre[2] + this->delta_t * x_pre[3];
        x_cur[3] = x_pre[3];
        x_cur[4] = x_pre[4] + this->delta_t * x_pre[5];
        x_cur[5] = x_pre[5];
    }

private:
    double delta_t = 0.;
};

template<typename T>
void ceres_xyz_to_ypd(const T xyz[Z_N_], T ypd[Z_N_]) {
    ypd[0] = ceres::atan2(xyz[1], xyz[0]); // yaw
    ypd[1] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1])); // pitch
    ypd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]); // distance
};
class EkfMeasure {
public:
    template<typename T>
    void operator()(const T x[X_N_], T y[Z_N_]) const {
        T x0[3] { x[0], x[2], x[4] };
        ceres_xyz_to_ypd(x0, y);
    }
};

using ArmorStateEKF = ExtendedKalmanFilter<X_N_, Z_N_, EkfPredict, EkfMeasure>;

} // namespace qd::auto_aim
#endif
