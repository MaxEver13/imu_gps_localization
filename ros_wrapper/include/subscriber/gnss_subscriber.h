/*
 * @Description: 订阅GNSS数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#ifndef IMU_GPS_LOCALIZATION_GNSS_SUBSCRIBER_H
#define IMU_GPS_LOCALIZATION_GNSS_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <memory>
#include <deque>
#include <mutex>
#include <string>

#include "sensor_data/gnss_data.h"


namespace ImuGpsLocalization {

class GnssSubscriber {
public:
  GnssSubscriber(ros::NodeHandle nh, std::string topic_name, size_t buff_size);
  ~GnssSubscriber() {};
  
  void ParseData(std::deque<GnssDataPtr>& deque_gnss_data);

private:
  void msg_callback(const sensor_msgs::NavSatFixConstPtr& gnss_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<GnssDataPtr> gnss_data_buffer_;
  std::mutex buffer_mutex_;


};

}


#endif