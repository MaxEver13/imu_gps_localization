/*
 * @Description: imu数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#ifndef IMU_GPS_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_H
#define IMU_GPS_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_H

#include <memory>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace ImuGpsLocalization {

class VelocityData
{  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp;
  Eigen::Vector3d vel;
  Eigen::Matrix3d cov;

  using VelocityDataPtr = std::shared_ptr<VelocityData>;
  using VelocityDataConstPtr = std::shared_ptr<const VelocityData>;

public:
  static bool SyncData(std::deque<VelocityDataPtr>& unSyncedData, std::deque<VelocityDataPtr>& syncedData, double sync_time);

};

using VelocityDataPtr = std::shared_ptr<VelocityData>;
using VelocityDataConstPtr = std::shared_ptr<const VelocityData>;

}

#endif