/*
 * @Description: imu,gnss融合定位
 * @Author: Ji Jiawen
 * @Date: 2020-04-06
 */

#ifndef IMU_GPS_LOCALIZATION_IMU_GPS_LOCALIZER_H
#define IMU_GPS_LOCALIZATION_IMU_GPS_LOCALIZER_H

#include <Eigen/Core>

#include "state.h"
#include "gps_processor.h"
#include "imu_processor.h"
#include "initializer.h"

namespace ImuGpsLocalization {

const double kGpsVelLimit = 3.; // m/s

class ImuGpsLocalizer {
public:
  ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                  const double acc_bias_noise, const double gyro_bias_noise,
                  const Eigen::Vector3d& I_p_Gps);

  bool ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state);

  bool ProcessGnssData(const GnssDataPtr gnss_data_ptr);

  bool ProcessVelData(const VelocityDataPtr vel_data_ptr);
    
private:
  std::unique_ptr<Initializer> initializer_;
  std::unique_ptr<ImuProcessor> imu_processor_;
  std::unique_ptr<GpsProcessor> gps_processor_;

  bool initialized_;
  Eigen::Vector3d init_lla_; // The initial reference gps point.
  State state_;
};

} 


#endif