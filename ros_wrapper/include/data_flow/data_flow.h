/*
 * @Description: 数据预处理
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#ifndef IMU_GPS_LOCALIZATION_DATA_FLOW_H
#define IMU_GPS_LOCALIZATION_DATA_FLOW_H

#include <ros/ros.h>

// Subscriber
#include "subscriber/gnss_subscriber.h"
#include "subscriber/imu_subscriber.h"
#include "subscriber/velocity_subscriber.h"

// Publisher
#include "publisher/gnss_publisher.h"
#include "publisher/imu_publisher.h"
#include "publisher/velocity_publisher.h"


namespace ImuGpsLocalization {

class DataFlow
{
public:
  DataFlow(ros::NodeHandle nh);
  ~DataFlow() {};

  bool Run();

private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool PublishData();


private:
  // Subscriber
  std::shared_ptr<GnssSubscriber> gnss_sub_ptr_;
  std::shared_ptr<ImuSubscriber> imu_sub_ptr_;
  std::shared_ptr<VelocitySubscriber> vel_sub_ptr_;

  // Publisher
  std::shared_ptr<GnssPublisher> gnss_pub_ptr_;
  std::shared_ptr<ImuPublisher> imu_pub_ptr_;
  std::shared_ptr<VelocityPublisher> vel_pub_ptr_;

  // Data buffer
  std::deque<GnssDataPtr> gnss_data_buff_;
  std::deque<ImuDataPtr> imu_data_buff_;
  std::deque<VelocityDataPtr> vel_data_buff_;

  // Current data(synced data)
  GnssDataPtr curr_gnss_data_ptr_;
  ImuDataPtr curr_imu_data_ptr_;
  VelocityDataPtr curr_vel_data_ptr_;

};



}



#endif