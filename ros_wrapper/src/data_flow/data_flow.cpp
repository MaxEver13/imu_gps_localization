/*
 * @Description: 数据流同步
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#include "data_flow/data_flow.h"
#include <iostream>

namespace ImuGpsLocalization {

DataFlow::DataFlow(ros::NodeHandle nh) {
  // subscriber
  gnss_sub_ptr_ = std::make_shared<GnssSubscriber>(nh, "/fix", 1000000);
  imu_sub_ptr_ = std::make_shared<ImuSubscriber>(nh, "/imu/data", 1000000);
  vel_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/vel", 1000000);

  // publisher
  gnss_pub_ptr_ = std::make_shared<GnssPublisher>(nh, "/synced_gnss", "navsat", 100);
  imu_pub_ptr_ = std::make_shared<ImuPublisher>(nh, "/synced_imu", "/base_imu", 100);
  vel_pub_ptr_ = std::make_shared<VelocityPublisher>(nh, "/synced_vel", "navsat", 100);
}

bool DataFlow::Run() {
  if (!ReadData()) {
    return false;
  }

  while (HasData()) {
    if (!ValidData())
      continue;

    PublishData();
  }

  return true;
}

// 读取gnss,imu,vel数据，并以gnss的时间戳同步imu和vel数据
bool DataFlow::ReadData() {
  // gnss数据不需要同步，直接拿到数据放在buffer中
  gnss_sub_ptr_->ParseData(gnss_data_buff_); 

  // 未同步的imu和vel数据，保存成静态变量，一直记录
  static std::deque<ImuDataPtr> unsynced_imu_buff;
  static std::deque<VelocityDataPtr> unsynced_vel_buff;
  imu_sub_ptr_->ParseData(unsynced_imu_buff);
  vel_sub_ptr_->ParseData(unsynced_vel_buff);

  // 没有gnss数据
  if (gnss_data_buff_.size() == 0)
    return false;

  // 同步imu和vel数据
  double gnss_time = gnss_data_buff_.front()->timestamp;
  bool valid_imu = ImuData::SyncData(unsynced_imu_buff, imu_data_buff_, gnss_time);
  bool valid_vel = VelocityData::SyncData(unsynced_vel_buff, vel_data_buff_, gnss_time);

  // 一旦当前gnss，imu，vel数据同步后，sensor_inited标志位为true
  // 如果没有同步的imu和vel数据，那么gnss数据也要丢掉，保证三个buffer里面
  // 均是同步后的数据
  static bool sensor_inited = false;
  if (!sensor_inited) {
    if (!valid_imu || !valid_vel) {
      gnss_data_buff_.pop_front();
      return false;
    }
    sensor_inited = true;
  }

  return true;
}

// 同时有同步后的所有数据
bool DataFlow::HasData() {
  if (gnss_data_buff_.size() == 0)
    return false;
  if (imu_data_buff_.size() == 0)
    return false;
  if (vel_data_buff_.size() == 0)
    return false;

  return true;
}

// 按照时间先后顺序，依次发布同步后的数据
bool DataFlow::ValidData() {
  curr_gnss_data_ptr_ = gnss_data_buff_.front();
  curr_imu_data_ptr_ = imu_data_buff_.front();
  curr_vel_data_ptr_ = vel_data_buff_.front();


  double diff_imu_time = curr_gnss_data_ptr_->timestamp - curr_imu_data_ptr_->timestamp;
  double diff_vel_time = curr_gnss_data_ptr_->timestamp - curr_vel_data_ptr_->timestamp;
  // 如果gnss数据时间比imu或者vel数据提前，并且超过50ms
  // 把gnss数据丢掉第一帧，数据不同步，不能发布
  if (diff_imu_time < -0.05 || diff_vel_time < -0.05) {
    gnss_data_buff_.pop_front();
    return false;
  }

  // 如果gnss数据比imu数据滞后，超过50ms
  // imu数据需要丢掉第一帧，数据不同步，不能发布
  if (diff_imu_time > 0.05) {
    imu_data_buff_.pop_front();
    return false;
  }

  // 如果gnss数据比vel数据滞后，超过50ms
  // vel数据需要丢掉第一帧，数据不同步，不能发布
  if (diff_vel_time > 0.05) {
    vel_data_buff_.pop_front();
    return false;
  }

  // 到这里，说明当前数据同步在50ms以内，认为可以发布
  // 并把缓冲区第一帧丢掉
  gnss_data_buff_.pop_front();
  imu_data_buff_.pop_front();
  vel_data_buff_.pop_front();

  return true;
}

// 发布同步后的数据
bool DataFlow::PublishData() {
  // std::cout << "gnss timestamp: " << std::fixed << curr_gnss_data_ptr_->timestamp << std::endl;
  // std::cout << "imu timestamp: " << std::fixed << curr_imu_data_ptr_->timestamp << std::endl;
  // std::cout << "vel timestamp: " << std::fixed << curr_vel_data_ptr_->timestamp << std::endl;
  gnss_pub_ptr_->Publish(curr_gnss_data_ptr_, curr_gnss_data_ptr_->timestamp);
  imu_pub_ptr_->Publish(curr_imu_data_ptr_, curr_imu_data_ptr_->timestamp);
  vel_pub_ptr_->Publish(curr_vel_data_ptr_, curr_vel_data_ptr_->timestamp);

  return true;
}
  
}

