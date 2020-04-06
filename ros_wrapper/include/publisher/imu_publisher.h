/*
 * @Description: 发布imu数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#ifndef IMU_GPS_LOCALIZATION_IMU_PUBLISHER_H
#define IMU_GPS_LOCALIZATION_IMU_PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>

#include "sensor_data/imu_data.h"

namespace ImuGpsLocalization {

class ImuPublisher
{
public:
  ImuPublisher(ros::NodeHandle nh, std::string topic_name, std::string frame_id, size_t buff_size);
  ~ImuPublisher() {};
  void Publish(ImuDataConstPtr imu_data_ptr, double time);

  bool HasSubscribers();

private:
  void PublishData(ImuDataConstPtr imu_data_ptr, ros::Time time);

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
  sensor_msgs::Imu imu_msg_;

};


}

#endif