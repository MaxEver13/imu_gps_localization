/*
 * @Description: 发布速度消息
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#ifndef IMU_GPS_LOCALIZATION_VELOCITY_PUBLISHER_H
#define IMU_GPS_LOCALIZATION_VELOCITY_PUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>

#include "sensor_data/velocity_data.h"

namespace ImuGpsLocalization {

class VelocityPublisher
{
public:
  VelocityPublisher(ros::NodeHandle nh, std::string topic_name, std::string frame_id, size_t buff_size);
  ~VelocityPublisher() {};

  void Publish(VelocityDataConstPtr velo_data_ptr, double time);

  bool HasSubscribers();

private:
  void PublishData(VelocityDataConstPtr velo_data_ptr, ros::Time time);

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
  geometry_msgs::TwistStamped velo_msg_;

};


}



#endif