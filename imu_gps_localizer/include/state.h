/*
 * @Description: ESKF状态
 * @Author: Ji Jiawen
 * @Date: 2020-04-06
 */

#ifndef IMU_GPS_LOCALIZATION_STATE_H
#define IMU_GPS_LOCALIZATION_STATE_H


#include <Eigen/Core>
#include "sensor_data/imu_data.h"

namespace ImuGpsLocalization {
    
struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp;
  
  Eigen::Vector3d lla;       // WGS84 position.
  Eigen::Vector3d G_p_I;     // The original point of the IMU frame in the Global frame.
  Eigen::Vector3d G_v_I;     // The velocity original point of the IMU frame in the Global frame.
  Eigen::Matrix3d G_R_I;     // The rotation from the IMU frame to the Global frame.
  Eigen::Vector3d acc_bias;  // The bias of the acceleration sensor.
  Eigen::Vector3d gyro_bias; // The bias of the gyroscope sensor.

  // Covariance.
  Eigen::Matrix<double, 15, 15> cov;

  // The imu data.
  ImuDataPtr imu_data_ptr; 
};

}


#endif