/*
 * @Description: imu数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#ifndef IMU_GPS_LOCALIZATION_SENSOR_DATA_GNSS_DATA_H
#define IMU_GPS_LOCALIZATION_SENSOR_DATA_GNSS_DATA_H

#include <memory>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace ImuGpsLocalization {

class GnssData
{  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  // 后续数据同步是以GNSS数据作为基准，将imu和velocity同步，因此这GNSS不需要数据同步接口
  // static bool SyncData(std::deque<GnssData> unSyncedData, std::deque<GnssData> syncedData, double sync_time);


  double timestamp;     // In second. 
  Eigen::Vector3d lla;  // Latitude in degree, longitude in degree, and altitude in meter.
  Eigen::Matrix3d cov;  // Covariance in m^2.



};

using GnssDataPtr = std::shared_ptr<GnssData>;
using GnssDataConstPtr = std::shared_ptr<const GnssData>;

}

#endif