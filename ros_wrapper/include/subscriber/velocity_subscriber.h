/*
 * @Description: 订阅imu数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#ifndef IMU_GPS_LOCALIZATION_VELOCITY_SUBSCRIBER_H
#define IMU_GPS_LOCALIZATION_VELOCITY_SUBSCRIBER_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <memory>
#include <deque>
#include <mutex>
#include <string>

#include "sensor_data/velocity_data.h"

namespace ImuGpsLocalization {

class VelocitySubscriber
{

public:
  VelocitySubscriber(ros::NodeHandle nh, std::string topic_name, size_t buff_size);
  ~VelocitySubscriber() {};

  void ParseData(std::deque<VelocityDataPtr>& deque_velo_data);

private:
  void msg_callback(const geometry_msgs::TwistStampedConstPtr& velo_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<VelocityDataPtr> velo_data_buffer_;
  std::mutex buffer_mutex_;

};




}

#endif