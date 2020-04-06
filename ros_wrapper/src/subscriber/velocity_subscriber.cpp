/*
 * @Description: 订阅imu数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#include "subscriber/velocity_subscriber.h"
#include <iostream>


namespace ImuGpsLocalization {

VelocitySubscriber::VelocitySubscriber(ros::NodeHandle nh, std::string topic_name, size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::msg_callback, this);
}

void VelocitySubscriber::ParseData(std::deque<VelocityDataPtr>& deque_velo_data) {
  buffer_mutex_.lock();
  if (velo_data_buffer_.size() > 0) {
    deque_velo_data.insert(deque_velo_data.end(), velo_data_buffer_.begin(), velo_data_buffer_.end());
    velo_data_buffer_.clear();
  }
  buffer_mutex_.unlock();
}

void VelocitySubscriber::msg_callback(const geometry_msgs::TwistStampedConstPtr& velo_msg_ptr) {
  buffer_mutex_.lock();
  VelocityDataPtr velo_data_ptr = std::make_shared<VelocityData>();

  velo_data_ptr->timestamp = velo_msg_ptr->header.stamp.toSec();
  velo_data_ptr->vel(0) = velo_msg_ptr->twist.linear.x;
  velo_data_ptr->vel(1) = velo_msg_ptr->twist.linear.y;
  velo_data_ptr->vel(2) = velo_msg_ptr->twist.linear.z;
  velo_data_ptr->cov = 2. * 2. * Eigen::Matrix3d::Identity();
  velo_data_ptr->cov(2, 2) = 1e10;

  // std::cout << "[VelocitySubscriber::msg_callback]: " << std::fixed << "timestamp: " << velo_data_ptr->timestamp << std::endl;  
  // std::cout << "vel: " << velo_data_ptr->vel << std::endl;  

  velo_data_buffer_.push_back(velo_data_ptr);
  buffer_mutex_.unlock();
}

}