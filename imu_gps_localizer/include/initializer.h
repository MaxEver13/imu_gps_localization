/*
 * @Description: imu,gnss融合定位
 * @Author: Ji Jiawen
 * @Date: 2020-04-06
 */

#ifndef IMU_GPS_LOCALIZATION_INITIALIZER_H
#define IMU_GPS_LOCALIZATION_INITIALIZER_H

#include <deque>

#include "state.h"

#include "sensor_data/imu_data.h"
#include "sensor_data/gnss_data.h"
#include "sensor_data/velocity_data.h"

namespace ImuGpsLocalization {

constexpr int kImuDataBufferLength = 100;
constexpr int kAccStdLimit         = 3.;

class Initializer {
public:
  Initializer(const Eigen::Vector3d& init_I_p_Gps);
  
  void AddImuData(const ImuDataPtr imu_data_ptr);

  bool AddGnssData(const GnssDataPtr gnss_data_ptr, State* state);
  
  void AddVelData(const VelocityDataPtr vel_data_ptr);

private:
  bool ComputeG_R_IFromImuData(Eigen::Matrix3d* G_R_I);


private:
  std::deque<ImuDataPtr> imu_buffer_;
  std::deque<VelocityDataPtr> vel_buffer_;
  
  Eigen::Vector3d init_I_p_Gps_;

};

}  

#endif