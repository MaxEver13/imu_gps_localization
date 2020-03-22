#pragma once

#include "imu_gps_localizer/base_type.h"

namespace ImuGpsLocalization {

class ImuProcessor{
public:
    ImuProcessor(const double acc_noise, const double gyro_noise,
                 const double acc_bias_noise, const double gyro_bias_noise,
                 const Eigen::Vector3d& gravity);

    void Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state);

private:
    void PredictNewState(const double& dt, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc,
                                                                        State* state);
    const double acc_noise_;
    const double gyro_noise_;
    const double acc_bias_noise_;
    const double gyro_bias_noise_;

    const Eigen::Vector3d gravity_;
};

}  // namespace ImuGpsLocalization