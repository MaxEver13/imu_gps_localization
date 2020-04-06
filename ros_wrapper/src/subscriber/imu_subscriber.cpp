/*
 * @Description: 订阅imu数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#include "subscriber/imu_subscriber.h"
#include <iostream>

namespace ImuGpsLocalization {
ImuSubscriber::ImuSubscriber(ros::NodeHandle nh, std::string topic_name, size_t buff_size) :  nh_(nh) { 
  subscriber_ = nh_.subscribe(topic_name, buff_size, &ImuSubscriber::msg_callback, this);
}

void ImuSubscriber::ParseData(std::deque<ImuDataPtr>& deque_imu_data) {
  buffer_mutex_.lock();
  if (imu_data_buffer_.size() > 0) {
    deque_imu_data.insert(deque_imu_data.end(), imu_data_buffer_.begin(), imu_data_buffer_.end());
    imu_data_buffer_.clear();
  }
  buffer_mutex_.unlock();
}

void ImuSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
  buffer_mutex_.lock();
  ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();

  imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
  imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                        imu_msg_ptr->linear_acceleration.y,
                        imu_msg_ptr->linear_acceleration.z;
  imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                        imu_msg_ptr->angular_velocity.y,
                        imu_msg_ptr->angular_velocity.z;

  // std::cout << "[ImuSubscriber::msg_callback]: " << std::fixed << "timestamp: " << imu_data_ptr->timestamp << std::endl;  
  // std::cout << "acc: " << imu_data_ptr->acc << std::endl;  
  // std::cout << "gyro: " << imu_data_ptr->gyro << std::endl;    

  imu_data_buffer_.push_back(imu_data_ptr);
  buffer_mutex_.unlock();                        
}


}