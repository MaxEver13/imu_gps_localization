/*
 * @Description: imu,gnss融合定位
 * @Author: Ji Jiawen
 * @Date: 2020-04-06
 */

#ifndef IMU_GPS_LOCALIZATION_LOCALIZATION_WRAPPER_H
#define IMU_GPS_LOCALIZATION_LOCALIZATION_WRAPPER_H

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>

#include "imu_gps_localizer.h"

namespace ImuGpsLocalization {

class LocalizationWrapper
{
public:
  LocalizationWrapper(ros::NodeHandle nh);
  ~LocalizationWrapper() {};


private:
  // message callback
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
  void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);
  void GpsVelocityCallback(const geometry_msgs::TwistStampedConstPtr& gps_vel_msg_ptr);

  void ConvertStateToRosTopic(const State& state);

private:
  // localizer
  std::unique_ptr<ImuGpsLocalizer> imu_gps_localizer_ptr_;

  // subscriber
  ros::Subscriber gnss_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber vel_sub_;

  // publisher
  ros::Publisher path_pub_;

  // fused path
  nav_msgs::Path path_;

};


}


#endif