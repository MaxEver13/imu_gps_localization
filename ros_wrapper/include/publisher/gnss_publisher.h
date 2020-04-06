/*
 * @Description: 发布gnss消息
 * @Author: Ji Jiawen
 * @Date: 2020-04-05
 */

#ifndef IMU_GPS_LOCALIZATION_GNSS_PUBLISHER_H
#define IMU_GPS_LOCALIZATION_GNSS_PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>

#include "sensor_data/gnss_data.h"

namespace ImuGpsLocalization {

class GnssPublisher
{
public:
  GnssPublisher(ros::NodeHandle nh, std::string topic_name, std::string frame_id, size_t buff_size);
  ~GnssPublisher() {};

  void Publish(GnssDataConstPtr gnss_data_ptr, double time);

  bool HasSubscribers();

private:
  void PublishData(GnssDataConstPtr gnss_data_ptr, ros::Time time);

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
  sensor_msgs::NavSatFix gnss_msg_;
};


}


#endif