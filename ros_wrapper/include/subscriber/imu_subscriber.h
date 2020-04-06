/*
 * @Description: 订阅imu数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#ifndef IMU_GPS_LOCALIZATION_IMU_SUBSCRIBER_H
#define IMU_GPS_LOCALIZATION_IMU_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "sensor_data/imu_data.h"

namespace ImuGpsLocalization {

class ImuSubscriber {
public:
  ImuSubscriber(ros::NodeHandle nh, std::string topic_name, size_t buff_size);
  ~ImuSubscriber() {};
  void ParseData(std::deque<ImuDataPtr>& deque_imu_data);

private:
  void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<ImuDataPtr> imu_data_buffer_;
  std::mutex buffer_mutex_;
};

}

#endif