/*
 * @Description: imu数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#ifndef IMU_GPS_LOCALIZATION_SENSOR_DATA_IMU_DATA_H
#define IMU_GPS_LOCALIZATION_SENSOR_DATA_IMU_DATA_H

#include <memory>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace ImuGpsLocalization {

class ImuData
{  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp;      // In second.
  Eigen::Vector3d acc;   // Acceleration in m/s^2
  Eigen::Vector3d gyro;  // Angular velocity in radian/s.

  using ImuDataPtr = std::shared_ptr<ImuData>;
  using ImuDataConstPtr = std::shared_ptr<const ImuData>;

public:
  static bool SyncData(std::deque<ImuDataPtr>& unSyncedData, std::deque<ImuDataPtr>& syncedData, double sync_time);
  
};

using ImuDataPtr = std::shared_ptr<ImuData>;
using ImuDataConstPtr = std::shared_ptr<const ImuData>;

}

#endif